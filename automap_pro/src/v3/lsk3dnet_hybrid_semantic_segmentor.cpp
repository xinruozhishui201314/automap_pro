/**
 * @file v3/lsk3dnet_hybrid_semantic_segmentor.cpp
 * @brief V3 流水线模块实现。
 */
#include "automap_pro/v3/semantic_segmentor.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <exception>
#include <filesystem>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <fcntl.h>
#include <fstream>
#include <sstream>
#include <signal.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <nlohmann/json.hpp>

#ifdef USE_TORCH
#include <torch/script.h>
#include <torch/torch.h>
#include <torch/version.h>
#endif

#include <rclcpp/rclcpp.hpp>
#include <shared_mutex>

namespace automap_pro::v3 {

namespace {
std::atomic<bool>& hybridShutdownGuard() {
    static std::atomic<bool> g{false};
    return g;
}
}  // namespace

void SetSemanticHybridShutdownGuard(bool enabled) {
    hybridShutdownGuard().store(enabled, std::memory_order_relaxed);
}

bool IsSemanticHybridShutdownGuard() {
    return hybridShutdownGuard().load(std::memory_order_relaxed);
}

#ifdef USE_TORCH
namespace detail_hybrid {

void appendNestedException(std::ostringstream& oss, const std::exception& e, int depth) {
    oss << "[depth=" << depth << "] " << e.what();
    try {
        std::rethrow_if_nested(e);
    } catch (const std::exception& nested) {
        oss << " | ";
        appendNestedException(oss, nested, depth + 1);
    } catch (...) {
        oss << " | [depth=" << (depth + 1) << "] <non-std nested exception>";
    }
}

std::string describeExceptionFull(const std::exception& e) {
    std::ostringstream oss;
    appendNestedException(oss, e, 0);
    return oss.str();
}

std::string describeCurrentExceptionFull() {
    try {
        throw;
    } catch (const std::exception& e) {
        return describeExceptionFull(e);
    } catch (...) {
        return "<non-std exception>";
    }
}

struct ProjectionIndex {
    int x = 0;
    int y = 0;
    bool valid = false;
};

inline ProjectionIndex projectPoint(const pcl::PointXYZI& p, int img_w, int img_h, double fov_up_rad, double fov_down_rad) {
    ProjectionIndex idx;
    const double range = std::sqrt(static_cast<double>(p.x) * p.x + static_cast<double>(p.y) * p.y + static_cast<double>(p.z) * p.z);
    if (!(range > 1e-6)) return idx;
    const double yaw = -std::atan2(static_cast<double>(p.y), static_cast<double>(p.x));
    const double sin_pitch = std::clamp(static_cast<double>(p.z) / range, -1.0, 1.0);
    const double pitch = std::asin(sin_pitch);
    const double fov = std::abs(fov_up_rad) + std::abs(fov_down_rad);

    int proj_x = static_cast<int>(std::floor(0.5 * (yaw / M_PI + 1.0) * static_cast<double>(img_w)));
    int proj_y = static_cast<int>(std::floor((1.0 - (pitch + std::abs(fov_down_rad)) / std::max(1e-6, fov)) * static_cast<double>(img_h)));
    proj_x = std::clamp(proj_x, 0, img_w - 1);
    proj_y = std::clamp(proj_y, 0, img_h - 1);
    idx.x = proj_x;
    idx.y = proj_y;
    idx.valid = true;
    return idx;
}

constexpr uint32_t kMagic = 0x314b534c;  // 'LSK1' little-endian
constexpr uint32_t kProtoVer = 1;
constexpr uint32_t kCmdInit = 1;
constexpr uint32_t kCmdInfer = 2;
constexpr uint32_t kCmdShutdown = 3;
constexpr uint32_t kErrResponse = 0xFFFFFFFFu;

void writeAll(int fd, const void* data, size_t n) {
    const auto* p = static_cast<const uint8_t*>(data);
    size_t off = 0;
    while (off < n) {
        const ssize_t w = ::write(fd, p + off, n - off);
        if (w < 0) {
            if (errno == EINTR) continue;
            throw std::runtime_error(std::string("hybrid worker write failed: ") + std::strerror(errno));
        }
        if (w == 0) throw std::runtime_error("hybrid worker write returned 0");
        off += static_cast<size_t>(w);
    }
}

void readAll(int fd, void* data, size_t n) {
    auto* p = static_cast<uint8_t*>(data);
    size_t off = 0;
    while (off < n) {
        const ssize_t r = ::read(fd, p + off, n - off);
        if (r < 0) {
            if (errno == EINTR) continue;
            throw std::runtime_error(std::string("hybrid worker read failed: ") + std::strerror(errno));
        }
        if (r == 0) throw std::runtime_error("hybrid worker EOF");
        off += static_cast<size_t>(r);
    }
}

void writeFrame(int fd, uint32_t cmd, const void* payload, uint64_t len) {
    const uint32_t magic = kMagic;
    writeAll(fd, &magic, 4);
    writeAll(fd, &kProtoVer, 4);
    writeAll(fd, &cmd, 4);
    writeAll(fd, &len, 8);
    if (len > 0) writeAll(fd, payload, static_cast<size_t>(len));
}

void readFrame(int fd, uint32_t* out_cmd, std::vector<uint8_t>* payload) {
    uint32_t magic = 0;
    readAll(fd, &magic, 4);
    if (magic != kMagic) throw std::runtime_error("hybrid worker bad magic");
    uint32_t ver = 0;
    readAll(fd, &ver, 4);
    if (ver != kProtoVer) throw std::runtime_error("hybrid worker bad protocol version");
    uint32_t cmd = 0;
    readAll(fd, &cmd, 4);
    uint64_t len = 0;
    readAll(fd, &len, 8);
    payload->resize(static_cast<size_t>(len));
    if (len > 0) readAll(fd, payload->data(), static_cast<size_t>(len));
    *out_cmd = cmd;
}

struct ClassifierMetaFile {
    bool present = false;
    uint32_t feat_dim = 0;
    uint32_t num_classes = 0;
    uint32_t input_dims = 0;
    std::string checkpoint_sha256_hex;
    std::string config_yaml_export;
};

static std::string jsonEscape(std::string s) {
    std::string o;
    o.reserve(s.size() + 8);
    for (char ch : s) {
        if (ch == '\\' || ch == '"') o.push_back('\\');
        o.push_back(ch);
    }
    return o;
}

ClassifierMetaFile loadClassifierMeta(const std::string& classifier_torchscript_path) {
    ClassifierMetaFile out;
    const std::string path = classifier_torchscript_path + ".meta.json";
    std::ifstream ifs(path);
    if (!ifs) return out;
    try {
        nlohmann::json j;
        ifs >> j;
        out.present = true;
        if (j.contains("feat_dim")) {
            out.feat_dim = static_cast<uint32_t>(j.at("feat_dim").get<int>());
        }
        if (j.contains("num_classes")) {
            out.num_classes = static_cast<uint32_t>(j.at("num_classes").get<int>());
        }
        if (j.contains("input_dims")) {
            out.input_dims = static_cast<uint32_t>(j.at("input_dims").get<int>());
        }
        if (j.contains("checkpoint_sha256_hex")) {
            out.checkpoint_sha256_hex = j.at("checkpoint_sha256_hex").get<std::string>();
        }
        if (j.contains("config_yaml_export")) {
            out.config_yaml_export = j.at("config_yaml_export").get<std::string>();
        }
    } catch (const std::exception& e) {
        throw std::runtime_error(std::string("lsk3dnet_hybrid: failed to parse classifier meta JSON: ") + path + ": " + e.what());
    }
    return out;
}

void validateTorchscriptClassifier(torch::jit::script::Module& mod, const torch::Device& dev, int feat_dim, int num_classes) {
    torch::NoGradGuard guard;
    auto x = torch::zeros({1, feat_dim}, torch::TensorOptions().dtype(torch::kFloat32).device(dev));
    auto out_iv = mod.forward({x});
    torch::Tensor out = out_iv.toTensor();
    if (!out.defined()) {
        throw std::runtime_error("lsk3dnet_hybrid: classifier TorchScript forward returned undefined tensor");
    }
    if (out.dim() != 2 || out.size(0) != 1 || out.size(1) != num_classes) {
        throw std::runtime_error("lsk3dnet_hybrid: classifier contract mismatch: need output [1," + std::to_string(num_classes) +
                                  "] got shape [" + std::to_string(out.size(0)) + "," + std::to_string(out.size(1)) + "]");
    }
}

static std::string buildInitJson(const SegmentorConfig& cfg, const std::string& expected_checkpoint_sha256_hex) {
    std::ostringstream js;
    js.setf(std::ios::fixed);
    js.precision(6);
    js << "{\"config_yaml\":\"" << jsonEscape(cfg.lsk3dnet_config_yaml) << "\","
       << "\"checkpoint\":\"" << jsonEscape(cfg.lsk3dnet_checkpoint) << "\","
       << "\"device\":\"" << jsonEscape(cfg.lsk3dnet_device) << "\","
       << "\"normal_mode\":\"" << jsonEscape(cfg.lsk3dnet_hybrid_normal_mode) << "\","
       << "\"normal_fov_up_deg\":" << cfg.lsk3dnet_normal_fov_up_deg << ","
       << "\"normal_fov_down_deg\":" << cfg.lsk3dnet_normal_fov_down_deg << ","
       << "\"normal_proj_h\":" << cfg.lsk3dnet_normal_proj_h << ","
       << "\"normal_proj_w\":" << cfg.lsk3dnet_normal_proj_w;
    if (!expected_checkpoint_sha256_hex.empty()) {
        js << ",\"expected_checkpoint_sha256\":\"" << jsonEscape(expected_checkpoint_sha256_hex) << "\"";
    }
    js << '}';
    return js.str();
}

class HybridPythonWorker final {
public:
    explicit HybridPythonWorker(const SegmentorConfig& cfg, const std::string& expected_checkpoint_sha256_hex) : cfg_(cfg) {
        int to_child[2]{-1, -1};
        int from_child[2]{-1, -1};
        if (pipe(to_child) != 0 || pipe(from_child) != 0) {
            throw std::runtime_error("hybrid worker pipe() failed");
        }
        pid_t pid = fork();
        if (pid < 0) {
            close_safe(to_child[0]);
            close_safe(to_child[1]);
            close_safe(from_child[0]);
            close_safe(from_child[1]);
            throw std::runtime_error("hybrid worker fork() failed");
        }
        if (pid == 0) {
            // child
            close_safe(to_child[1]);
            close_safe(from_child[0]);

            // 🏛️ [协议通道解耦] 必须极度小心地进行 FD 重定向，避免 FD 编号冲突导致的意外关闭。
            // 目标：to_child[0] -> FD 0, from_child[1] -> FD 3, FD 2 -> FD 1
            
            // 1. 将 stderr (2) 复制到 stdout (1)，确保日志流正确
            if (dup2(STDERR_FILENO, STDOUT_FILENO) < 0) _exit(121);

            // 2. 将输入 pipe 复制到 STDIN (0)
            if (to_child[0] != STDIN_FILENO) {
                if (dup2(to_child[0], STDIN_FILENO) < 0) _exit(120);
                // 仅当 original FD 不是目标集合 {0, 1, 2, 3} 时才安全关闭
                if (to_child[0] > 3) ::close(to_child[0]);
            }

            // 3. 将输出 pipe 复制到专用通道 FD 3
            if (from_child[1] != 3) {
                if (dup2(from_child[1], 3) < 0) _exit(121);
                // 仅当 original FD 不是目标集合 {0, 1, 2, 3} 时才安全关闭
                if (from_child[1] > 3) ::close(from_child[1]);
            }

            if (::chdir(cfg_.lsk3dnet_repo_root.c_str()) != 0) _exit(122);
            // 🏛️ [Fix] Add c_utils to PYTHONPATH so worker can find compiled normal generation libraries
            std::string python_path = cfg_.lsk3dnet_repo_root + ":" + 
                                     (std::filesystem::path(cfg_.lsk3dnet_repo_root) / "c_utils").string();
            const char* old_pp = std::getenv("PYTHONPATH");
            if (old_pp) python_path = std::string(old_pp) + ":" + python_path;
            ::setenv("PYTHONPATH", python_path.c_str(), 1);
            // 🏛️ [架构加固] 降低语义推理进程优先级，确保前端 LIO 实时性 (解决 IMU Jumps)
            nice(10);
            const std::string& py = cfg_.lsk3dnet_python_exe;
            
            // Log child startup info
            std::cerr << "[HybridPythonWorker] Child starting: " << py << " " << cfg_.lsk3dnet_worker_script << std::endl;
            
            execlp(py.c_str(), py.c_str(), "-u", cfg_.lsk3dnet_worker_script.c_str(), "--stdio", static_cast<char*>(nullptr));
            _exit(123);
        }
        close_safe(to_child[0]);
        close_safe(from_child[1]);
        fd_write_ = to_child[1];
        fd_read_ = from_child[0];
        pid_ = pid;

        RCLCPP_INFO(rclcpp::get_logger("automap_system"), 
                    "[SEMANTIC][HybridWorker] Spawned Python worker PID=%d", pid_);

        const std::string json = buildInitJson(cfg_, expected_checkpoint_sha256_hex);
        RCLCPP_INFO(rclcpp::get_logger("automap_system"), 
                    "[SEMANTIC][HybridWorker] Sending INIT JSON to worker...");
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[SEMANTIC][HybridWorker][INIT_AUDIT] source={SegmentorConfig from SemanticProcessor} "
                    "effective={repo_root=%s config_yaml=%s checkpoint=%s python=%s worker=%s device=%s "
                    "normal_mode=%s normal_profile=[up=%.1f,down=%.1f,h=%d,w=%d]}",
                    cfg_.lsk3dnet_repo_root.c_str(),
                    cfg_.lsk3dnet_config_yaml.c_str(),
                    cfg_.lsk3dnet_checkpoint.c_str(),
                    cfg_.lsk3dnet_python_exe.c_str(),
                    cfg_.lsk3dnet_worker_script.c_str(),
                    cfg_.lsk3dnet_device.c_str(),
                    cfg_.lsk3dnet_hybrid_normal_mode.c_str(),
                    cfg_.lsk3dnet_normal_fov_up_deg,
                    cfg_.lsk3dnet_normal_fov_down_deg,
                    cfg_.lsk3dnet_normal_proj_h,
                    cfg_.lsk3dnet_normal_proj_w);
        RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
                    "[SEMANTIC][HybridWorker][INIT_AUDIT] init_json=%s", json.c_str());
        
        writeFrame(fd_write_, kCmdInit, json.data(), json.size());

        std::vector<uint8_t> pl;
        uint32_t rsp = 0;
        readFrame(fd_read_, &rsp, &pl);
        if (rsp == kErrResponse) {
            std::string err_msg(pl.begin(), pl.end());
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"), "[SEMANTIC][HybridWorker] Python worker INIT FAILED: %s", err_msg.c_str());
            throw std::runtime_error("hybrid Python INIT failed: " + err_msg);
        }
        if (rsp != kCmdInit || pl.size() < 12) {
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"), "[SEMANTIC][HybridWorker] Python worker INIT unexpected response (cmd=%u, len=%zu)", rsp, pl.size());
            throw std::runtime_error("hybrid Python INIT unexpected response");
        }
        uint32_t id = 0, fd = 0, nc = 0;
        std::memcpy(&id, pl.data(), 4);
        std::memcpy(&fd, pl.data() + 4, 4);
        std::memcpy(&nc, pl.data() + 8, 4);
        input_dims_ = id;
        feat_dim_ = fd;
        num_classes_ = nc;
        
        RCLCPP_INFO(rclcpp::get_logger("automap_system"), 
                    "[SEMANTIC][HybridWorker] Python worker INIT OK: input_dims=%u feat_dim=%u num_classes=%u", 
                    input_dims_, feat_dim_, num_classes_);
    }

    ~HybridPythonWorker() { shutdown(); }

    HybridPythonWorker(const HybridPythonWorker&) = delete;
    HybridPythonWorker& operator=(const HybridPythonWorker&) = delete;

    void infer_features(const std::vector<float>& points_nc, std::vector<float>* out_feat) {
        const uint64_t n = points_nc.size() / static_cast<size_t>(input_dims_);
        if (points_nc.size() != n * static_cast<size_t>(input_dims_)) {
            throw std::runtime_error("hybrid infer: bad point buffer size");
        }
        std::vector<uint8_t> req(12 + points_nc.size() * sizeof(float));
        const uint64_t nn = n;
        const uint32_t c = input_dims_;
        std::memcpy(req.data(), &nn, 8);
        std::memcpy(req.data() + 8, &c, 4);
        std::memcpy(req.data() + 12, points_nc.data(), points_nc.size() * sizeof(float));
        writeFrame(fd_write_, kCmdInfer, req.data(), req.size());

        std::vector<uint8_t> pl;
        uint32_t rsp = 0;
        readFrame(fd_read_, &rsp, &pl);
        if (rsp == kErrResponse) {
            throw std::runtime_error("hybrid Python INFER failed: " + std::string(pl.begin(), pl.end()));
        }
        if (rsp != kCmdInfer || pl.size() < 12) {
            throw std::runtime_error("hybrid Python INFER unexpected response");
        }
        uint64_t n2 = 0;
        uint32_t fdim = 0;
        std::memcpy(&n2, pl.data(), 8);
        std::memcpy(&fdim, pl.data() + 8, 4);
        if (n2 != n || fdim != feat_dim_) {
            throw std::runtime_error("hybrid INFER shape mismatch");
        }
        const size_t expect_body = 12 + static_cast<size_t>(n) * static_cast<size_t>(feat_dim_) * sizeof(float);
        if (pl.size() != expect_body) {
            throw std::runtime_error("hybrid INFER payload size mismatch");
        }
        out_feat->resize(static_cast<size_t>(n) * static_cast<size_t>(feat_dim_));
        std::memcpy(out_feat->data(), pl.data() + 12, out_feat->size() * sizeof(float));
    }

    int input_dims() const { return static_cast<int>(input_dims_); }
    int feat_dim() const { return static_cast<int>(feat_dim_); }
    int num_classes() const { return static_cast<int>(num_classes_); }

private:
    static void close_safe(int& fd) {
        if (fd >= 0) {
            ::close(fd);
            fd = -1;
        }
    }

    void shutdown() {
        if (fd_write_ >= 0) {
            try {
                writeFrame(fd_write_, kCmdShutdown, nullptr, 0);
            } catch (...) {
                RCLCPP_ERROR(
                    rclcpp::get_logger("automap_system"),
                    "[SEMANTIC][HybridWorker][SHUTDOWN] write shutdown frame failed: %s",
                    describeCurrentExceptionFull().c_str());
            }
            close_safe(fd_write_);
        }
        close_safe(fd_read_);
        if (pid_ > 0) {
            int st = 0;
            (void)waitpid(pid_, &st, 0);
            pid_ = -1;
        }
    }

    SegmentorConfig cfg_;
    int fd_write_ = -1;
    int fd_read_ = -1;
    pid_t pid_ = -1;
    uint32_t input_dims_ = 0;
    uint32_t feat_dim_ = 0;
    uint32_t num_classes_ = 0;
};

struct HybridSharedState {
    // 🏛️ [架构演进] 细粒度锁策略：分离 Python 骨干 (Backbone) 与 C++ 分类头 (Classifier)
    // 解决多线程环境下 LibTorch 前向传播阻塞 Python I/O 的问题
    std::mutex backbone_mu;
    std::unique_ptr<HybridPythonWorker> worker;

    std::shared_mutex classifier_mu;
    torch::jit::script::Module classifier;
    torch::Device device{torch::kCPU};
    
    std::string init_checkpoint;
    SegmentorConfig frozen_cfg;
    uint32_t validated_feat_dim = 0;
    uint32_t validated_num_classes = 0;
    bool classifier_initialized = false;
    
    // 🏛️ [逻辑加固] 统一 FOV 状态，解决 C++/Python 映射不一致问题
    double nn_fov_up_rad = 0.0;
    double nn_fov_down_rad = 0.0;

    // Recovery storm guard (protected by backbone_mu).
    std::chrono::steady_clock::time_point last_restart_tp{};
    uint64_t restart_attempts_total = 0;
    uint64_t restart_success_total = 0;
    uint64_t restart_throttled_total = 0;
};

HybridSharedState& hybridState() {
    static HybridSharedState s;
    return s;
}

bool isRecoverableWorkerTransportError(const std::string& msg) {
    return (msg.find("hybrid worker EOF") != std::string::npos) ||
           (msg.find("hybrid worker bad magic") != std::string::npos) ||
           (msg.find("hybrid worker bad protocol version") != std::string::npos) ||
           (msg.find("hybrid worker read failed") != std::string::npos) ||
           (msg.find("hybrid worker write failed") != std::string::npos);
}

enum class HybridRecoverStatus : int {
    kNoError = 0,
    kNonRecoverable = 1,
    kRecoverable = 2,
    kRestartThrottled = 3,
    kRestartSuccess = 4,
    kRestartFailed = 5,
};

const char* toString(HybridRecoverStatus st) {
    switch (st) {
        case HybridRecoverStatus::kNoError: return "NO_ERROR";
        case HybridRecoverStatus::kNonRecoverable: return "NON_RECOVERABLE";
        case HybridRecoverStatus::kRecoverable: return "RECOVERABLE";
        case HybridRecoverStatus::kRestartThrottled: return "RESTART_THROTTLED";
        case HybridRecoverStatus::kRestartSuccess: return "RESTART_SUCCESS";
        case HybridRecoverStatus::kRestartFailed: return "RESTART_FAILED";
    }
    return "UNKNOWN";
}

void hybridEnsureInit(const SegmentorConfig& cfg) {
    HybridSharedState& st = hybridState();
    
    // 1. 初始化 Python 骨干 (独占锁)
    {
        std::lock_guard<std::mutex> lock(st.backbone_mu);
        if (st.worker) {
            if (cfg.lsk3dnet_checkpoint != st.init_checkpoint) {
                throw std::runtime_error("lsk3dnet_hybrid: Checkpoint mismatch between threads");
            }
            // Do not return early here. Classifier init is guarded separately and
            // must still be observed as complete by concurrent constructors.
        } else {
            st.frozen_cfg = cfg;
            st.nn_fov_up_rad = static_cast<double>(cfg.fov_up) * M_PI / 180.0;
            st.nn_fov_down_rad = static_cast<double>(cfg.fov_down) * M_PI / 180.0;

            const ClassifierMetaFile meta = loadClassifierMeta(cfg.lsk3dnet_classifier_torchscript);
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][LSK3DNET_HYBRID][LOAD] step=begin ts=%s py=%s worker=%s repo_root=%s cfg=%s ckpt=%s cls_ts=%s dev_req=%s normal(mode=%s fov=[%.1f,%.1f] proj=%dx%d)",
                TORCH_VERSION, cfg.lsk3dnet_python_exe.c_str(), cfg.lsk3dnet_worker_script.c_str(),
                cfg.lsk3dnet_repo_root.c_str(), cfg.lsk3dnet_config_yaml.c_str(),
                cfg.lsk3dnet_checkpoint.c_str(), cfg.lsk3dnet_classifier_torchscript.c_str(),
                cfg.lsk3dnet_device.c_str(), cfg.lsk3dnet_hybrid_normal_mode.c_str(),
                cfg.lsk3dnet_normal_fov_up_deg, cfg.lsk3dnet_normal_fov_down_deg,
                cfg.lsk3dnet_normal_proj_w, cfg.lsk3dnet_normal_proj_h);

            st.worker = std::make_unique<HybridPythonWorker>(cfg, meta.checkpoint_sha256_hex);
            st.validated_feat_dim = static_cast<uint32_t>(st.worker->feat_dim());
            st.validated_num_classes = static_cast<uint32_t>(st.worker->num_classes());
            st.init_checkpoint = cfg.lsk3dnet_checkpoint;

            if (meta.present) {
                const std::string cfg_yaml = std::filesystem::weakly_canonical(cfg.lsk3dnet_config_yaml).string();
                std::string meta_yaml = meta.config_yaml_export;
                if (!meta_yaml.empty()) {
                    meta_yaml = std::filesystem::weakly_canonical(meta_yaml).string();
                }
                if (meta.num_classes > 0 && meta.num_classes != st.validated_num_classes) {
                    throw std::runtime_error(
                        "lsk3dnet_hybrid: classifier meta num_classes mismatch with worker runtime: meta=" +
                        std::to_string(meta.num_classes) + " runtime=" + std::to_string(st.validated_num_classes));
                }
                if (meta.feat_dim > 0 && meta.feat_dim != st.validated_feat_dim) {
                    throw std::runtime_error(
                        "lsk3dnet_hybrid: classifier meta feat_dim mismatch with worker runtime: meta=" +
                        std::to_string(meta.feat_dim) + " runtime=" + std::to_string(st.validated_feat_dim));
                }
                if (!meta_yaml.empty() && meta_yaml != cfg_yaml) {
                    RCLCPP_WARN(
                        rclcpp::get_logger("automap_system"),
                        "[SEMANTIC][LSK3DNET_HYBRID] meta config_yaml_export differs from runtime config_yaml (meta=%s runtime=%s)",
                        meta_yaml.c_str(), cfg_yaml.c_str());
                }
                RCLCPP_INFO(
                    rclcpp::get_logger("automap_system"),
                    "[SEMANTIC][LSK3DNET_HYBRID] meta check OK (feat_dim=%u num_classes=%u input_dims=%u sha256=%s)",
                    meta.feat_dim, meta.num_classes, meta.input_dims,
                    meta.checkpoint_sha256_hex.empty() ? "none" : "provided");
            } else {
                RCLCPP_WARN(
                    rclcpp::get_logger("automap_system"),
                    "[SEMANTIC][LSK3DNET_HYBRID] classifier meta not found; checkpoint/model consistency checks are limited");
            }
        }
    }

    // 2. 初始化 LibTorch 分类头 (独占锁)
    {
        std::unique_lock<std::shared_mutex> lock(st.classifier_mu);
        if (st.classifier_initialized) {
            return;
        }
        at::globalContext().setUserEnabledMkldnn(false);
        torch::set_num_threads(1);

        if (cfg.lsk3dnet_device.rfind("cuda", 0) == 0) {
            if (!torch::cuda::is_available()) {
                throw std::runtime_error("lsk3dnet_hybrid: CUDA device requested ('" + cfg.lsk3dnet_device + 
                                         "'), but torch::cuda::is_available() is false. Check CUDA/Driver installation.");
            }
        }

        try {
            st.classifier = torch::jit::load(cfg.lsk3dnet_classifier_torchscript);
            RCLCPP_INFO(rclcpp::get_logger("automap_system"), "[SEMANTIC][LSK3DNET_HYBRID] Successfully loaded TorchScript from %s", 
                        cfg.lsk3dnet_classifier_torchscript.c_str());
        } catch (const std::exception& e) {
            std::string err_msg = "lsk3dnet_hybrid: Failed to load TorchScript classifier from '" + 
                                  cfg.lsk3dnet_classifier_torchscript + "'. Error: " + e.what();
            if (!std::filesystem::exists(cfg.lsk3dnet_classifier_torchscript)) {
                err_msg += " (File does NOT exist)";
            } else {
                auto perms = std::filesystem::status(cfg.lsk3dnet_classifier_torchscript).permissions();
                err_msg += " (File exists, perms: " + std::to_string(static_cast<int>(perms)) + ")";
            }
            throw std::runtime_error(err_msg);
        }
        st.classifier.eval();

        if (cfg.lsk3dnet_device.rfind("cuda", 0) != 0) {
            RCLCPP_WARN(rclcpp::get_logger("automap_system"), 
                "[SEMANTIC][LSK3DNET_HYBRID] Using CPU for classifier (device=%s). This might be SLOW.", 
                cfg.lsk3dnet_device.c_str());
        }
        
        st.device = torch::Device(cfg.lsk3dnet_device);
        st.classifier.to(st.device);
        validateTorchscriptClassifier(st.classifier, st.device, static_cast<int>(st.validated_feat_dim),
                                      static_cast<int>(st.validated_num_classes));
        st.classifier_initialized = true;
        
        RCLCPP_INFO(rclcpp::get_logger("automap_system"), 
                    "[SEMANTIC][LSK3DNET_HYBRID] Classifier contract validation OK (feat_dim=%u, num_classes=%u, device=%s)",
                    st.validated_feat_dim, st.validated_num_classes, st.device.str().c_str());
    }
}

void inferFeaturesWithRecover(const std::vector<float>& pts, std::vector<float>* feat, torch::Device* out_dev) {
    HybridSharedState& st = hybridState();
    constexpr auto kRestartCooldown = std::chrono::milliseconds(1500);
    if (IsSemanticHybridShutdownGuard()) {
        throw std::runtime_error("hybrid recover blocked: shutdown guard is active");
    }
    
    // 🏛️ [架构加固] 仅在骨干推理时持有 backbone_mu，允许分类头并发
    std::lock_guard<std::mutex> lock(st.backbone_mu);
    auto run_infer = [&] {
        if (!st.worker) {
            throw std::runtime_error("hybrid worker unavailable before infer");
        }
        st.worker->infer_features(pts, feat);
    };
    try {
        run_infer();
    } catch (const std::runtime_error& e) {
        const std::string msg = e.what();
        const bool recoverable = isRecoverableWorkerTransportError(msg);
        RCLCPP_ERROR(
            rclcpp::get_logger("automap_system"),
            "[SEMANTIC][HybridRecover] infer exception: status=%s(%d) recoverable=%d msg=%s pts_floats=%zu feat_ptr=%p worker_present=%d",
            toString(recoverable ? HybridRecoverStatus::kRecoverable : HybridRecoverStatus::kNonRecoverable),
            static_cast<int>(recoverable ? HybridRecoverStatus::kRecoverable : HybridRecoverStatus::kNonRecoverable),
            recoverable ? 1 : 0,
            msg.c_str(),
            pts.size(),
            static_cast<void*>(feat),
            st.worker ? 1 : 0);
        if (recoverable) {
            if (IsSemanticHybridShutdownGuard()) {
                RCLCPP_ERROR(
                    rclcpp::get_logger("automap_system"),
                    "[SEMANTIC][HybridRecover] status=%s(%d) restart blocked by shutdown guard",
                    toString(HybridRecoverStatus::kRestartThrottled),
                    static_cast<int>(HybridRecoverStatus::kRestartThrottled));
                throw;
            }
            const auto now = std::chrono::steady_clock::now();
            if (st.last_restart_tp.time_since_epoch().count() != 0 &&
                (now - st.last_restart_tp) < kRestartCooldown) {
                ++st.restart_throttled_total;
                const auto wait_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    kRestartCooldown - (now - st.last_restart_tp)).count();
                RCLCPP_ERROR(
                    rclcpp::get_logger("automap_system"),
                    "[SEMANTIC][HybridRecover] status=%s(%d) cooldown_active wait_ms=%ld throttled_total=%lu reason=%s",
                    toString(HybridRecoverStatus::kRestartThrottled),
                    static_cast<int>(HybridRecoverStatus::kRestartThrottled),
                    static_cast<long>(wait_ms),
                    static_cast<unsigned long>(st.restart_throttled_total),
                    msg.c_str());
                throw;
            }
            st.last_restart_tp = now;
            ++st.restart_attempts_total;
            RCLCPP_WARN(
                rclcpp::get_logger("automap_system"),
                "[SEMANTIC][HybridRecover] status=%s(%d) restart worker attempt=%lu reason=%s",
                toString(HybridRecoverStatus::kRecoverable),
                static_cast<int>(HybridRecoverStatus::kRecoverable),
                static_cast<unsigned long>(st.restart_attempts_total),
                msg.c_str());
            st.worker.reset();
            try {
                const ClassifierMetaFile meta = loadClassifierMeta(st.frozen_cfg.lsk3dnet_classifier_torchscript);
                st.worker = std::make_unique<HybridPythonWorker>(st.frozen_cfg, meta.checkpoint_sha256_hex);
                RCLCPP_INFO(
                    rclcpp::get_logger("automap_system"),
                    "[SEMANTIC][HybridRecover] worker recreated successfully, retry infer now");
                run_infer();
                ++st.restart_success_total;
                RCLCPP_INFO(
                    rclcpp::get_logger("automap_system"),
                    "[SEMANTIC][HybridRecover] status=%s(%d) retry infer succeeded restart_success_total=%lu",
                    toString(HybridRecoverStatus::kRestartSuccess),
                    static_cast<int>(HybridRecoverStatus::kRestartSuccess),
                    static_cast<unsigned long>(st.restart_success_total));
            } catch (const std::exception& restart_err) {
                const std::string full_err = describeExceptionFull(restart_err);
                RCLCPP_FATAL(
                    rclcpp::get_logger("automap_system"),
                    "[SEMANTIC][HybridRecover] status=%s(%d) worker restart/retry failed: %s",
                    toString(HybridRecoverStatus::kRestartFailed),
                    static_cast<int>(HybridRecoverStatus::kRestartFailed),
                    restart_err.what());
                RCLCPP_FATAL(
                    rclcpp::get_logger("automap_system"),
                    "[SEMANTIC][HybridRecover] FULL_EXCEPTION: %s",
                    full_err.c_str());
                throw;
            }
        } else {
            RCLCPP_ERROR(
                rclcpp::get_logger("automap_system"),
                "[SEMANTIC][HybridRecover] status=%s(%d) non-recoverable infer error, rethrow",
                toString(HybridRecoverStatus::kNonRecoverable),
                static_cast<int>(HybridRecoverStatus::kNonRecoverable));
            throw;
        }
    } catch (const std::exception& e) {
        const std::string full_err = describeExceptionFull(e);
        RCLCPP_FATAL(
            rclcpp::get_logger("automap_system"),
            "[SEMANTIC][HybridRecover] non-runtime std::exception during infer: %s",
            full_err.c_str());
        throw;
    } catch (...) {
        RCLCPP_FATAL(
            rclcpp::get_logger("automap_system"),
            "[SEMANTIC][HybridRecover] unknown exception during infer: %s",
            describeCurrentExceptionFull().c_str());
        throw;
    }
    
    // 获取当前设备状态 (读锁)
    {
        std::shared_lock<std::shared_mutex> lock(st.classifier_mu);
        *out_dev = st.device;
    }
}

/** vote_idx==0: identity; else deterministic xy aug (flip / in-plane rotation). Index-preserving for logits sum. */
void applyDeterministicTtaToPts(std::vector<float>& pts, int c, int vote_idx) {
    if (vote_idx <= 0 || c < 3) return;
    const size_t n = pts.size() / static_cast<size_t>(c);
    const int op = (vote_idx - 1) % 5;
    for (size_t i = 0; i < n; ++i) {
        const size_t b = i * static_cast<size_t>(c);
        float x = pts[b + 0];
        float y = pts[b + 1];
        float z = pts[b + 2];
        switch (op) {
            case 0:
                x = -x;
                break;
            case 1:
                y = -y;
                break;
            case 2: {
                const float nx = -y;
                const float ny = x;
                x = nx;
                y = ny;
            } break;
            case 3:
                x = -x;
                y = -y;
                break;
            case 4: {
                const float nx = y;
                const float ny = -x;
                x = nx;
                y = ny;
            } break;
            default:
                break;
        }
        pts[b + 0] = x;
        pts[b + 1] = y;
        pts[b + 2] = z;
    }
}

}  // namespace detail_hybrid

class Lsk3dnetHybridSemanticSegmentor final : public ISemanticSegmentor {
public:
    explicit Lsk3dnetHybridSemanticSegmentor(const SegmentorConfig& cfg)
        : cfg_(cfg) {
        if (cfg_.lsk3dnet_repo_root.empty() || cfg_.lsk3dnet_config_yaml.empty() || cfg_.lsk3dnet_checkpoint.empty() ||
            cfg_.lsk3dnet_classifier_torchscript.empty() || cfg_.lsk3dnet_worker_script.empty()) {
            throw std::runtime_error("lsk3dnet_hybrid: missing assets");
        }
        detail_hybrid::hybridEnsureInit(cfg_);
        
        detail_hybrid::HybridSharedState& st = detail_hybrid::hybridState();
        // 🏛️ [架构演进] 从共享状态同步统一的 FOV 弧度，确保 Segmentor 内部一致性
        fov_up_rad_ = st.nn_fov_up_rad;
        fov_down_rad_ = st.nn_fov_down_rad;
        
        std::lock_guard<std::mutex> lock(st.backbone_mu);
        input_dims_ = st.worker->input_dims();
        feat_dim_ = st.worker->feat_dim();
    }

    const char* name() const override { return "lsk3dnet_hybrid"; }
    bool isReady() const override { return true; }

    void run(const CloudXYZIConstPtr& cloud, cv::Mat& mask, SemanticSegResult* result) override {
        if (!cloud || cloud->empty()) {
            mask = cv::Mat::zeros(cfg_.img_h, cfg_.img_w, CV_8U);
            if (result != nullptr) {
                result->per_point_labels.clear();
                result->success = true;
                result->backend_name = name();
                result->message = "empty cloud";
                result->inference_ms = 0.0;
            }
            return;
        }
        if (result != nullptr) {
            result->per_point_labels.clear();
        }
        const auto t0 = std::chrono::steady_clock::now();

        const int c = input_dims_;
        const size_t N = cloud->size();

        auto point_in_training_volume = [&](const pcl::PointXYZI& p) -> bool {
            if (!cfg_.lsk_training_volume_crop || !cfg_.lsk_volume_bounds_valid) {
                return true;
            }
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
                return false;
            }
            return p.x >= cfg_.lsk_vol_min_x && p.x <= cfg_.lsk_vol_max_x && p.y >= cfg_.lsk_vol_min_y &&
                   p.y <= cfg_.lsk_vol_max_y && p.z >= cfg_.lsk_vol_min_z && p.z <= cfg_.lsk_vol_max_z;
        };

        std::vector<size_t> orig_idx;
        orig_idx.reserve(N);
        for (size_t i = 0; i < N; ++i) {
            if (point_in_training_volume(cloud->points[i])) {
                orig_idx.push_back(i);
            }
        }

        if (orig_idx.empty()) {
            mask = cv::Mat::zeros(cfg_.img_h, cfg_.img_w, CV_8U);
            if (result != nullptr) {
                result->per_point_labels.assign(N, 0);
                result->success = true;
                result->backend_name = name();
                result->message = "no points in inference set (volume crop or empty)";
                result->inference_ms = 0.0;
            }
            return;
        }

        const size_t M = orig_idx.size();
        std::vector<float> pts_base(M * static_cast<size_t>(c), 0.0f);
        for (size_t k = 0; k < M; ++k) {
            const auto& p = cloud->points[orig_idx[k]];
            const size_t base = k * static_cast<size_t>(c);
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
                pts_base[base + 0] = 0.0f;
                pts_base[base + 1] = 0.0f;
                pts_base[base + 2] = 0.0f;
            } else {
                pts_base[base + 0] = p.x;
                pts_base[base + 1] = p.y;
                pts_base[base + 2] = p.z;
            }
            if (c > 3) {
                const float val = std::isfinite(p.intensity) ? p.intensity : 0.0f;
                pts_base[base + 3] = val;
            }
        }

        const int num_votes = std::max(1, std::min(32, cfg_.lsk_num_vote));
        std::vector<float> acc_logits;
        torch::Device infer_device{torch::kCPU};
        static std::atomic<uint64_t> run_call_idx{0};
        const uint64_t call_idx = run_call_idx.fetch_add(1, std::memory_order_relaxed) + 1;
        if (call_idx == 1) {
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Hybrid][FIRST_FRAME_SNAPSHOT] cloud_pts=%zu infer_pts=%zu num_vote=%d vol_crop=%d "
                "input_dims=%d feat_dim=%d normal_mode=%s normal_profile=[up=%.1f,down=%.1f,h=%d,w=%d] "
                "sensor_fov=[up=%.1f,down=%.1f] img=[%dx%d] tree_class=%d",
                N,
                M,
                num_votes,
                (cfg_.lsk_training_volume_crop && cfg_.lsk_volume_bounds_valid) ? 1 : 0,
                input_dims_,
                feat_dim_,
                cfg_.lsk3dnet_hybrid_normal_mode.c_str(),
                cfg_.lsk3dnet_normal_fov_up_deg,
                cfg_.lsk3dnet_normal_fov_down_deg,
                cfg_.lsk3dnet_normal_proj_h,
                cfg_.lsk3dnet_normal_proj_w,
                cfg_.fov_up,
                cfg_.fov_down,
                cfg_.img_w,
                cfg_.img_h,
                cfg_.tree_class_id);
        }

        torch::NoGradGuard no_grad;
        detail_hybrid::HybridSharedState& st = detail_hybrid::hybridState();
        int num_classes_runtime = 0;

        for (int vi = 0; vi < num_votes; ++vi) {
            std::vector<float> pts_work = pts_base;
            detail_hybrid::applyDeterministicTtaToPts(pts_work, c, vi);

            std::vector<float> feat;
            try {
                detail_hybrid::inferFeaturesWithRecover(pts_work, &feat, &infer_device);
            } catch (const std::exception& e) {
                const std::string full_err = detail_hybrid::describeExceptionFull(e);
                RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                    "[SEMANTIC][Hybrid] Feature extraction failed (likely worker crash or missing c_gen_normal_map): %s",
                    e.what());
                RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                    "[SEMANTIC][Hybrid] Feature extraction FULL_EXCEPTION: %s", full_err.c_str());
                if (result != nullptr) {
                    result->per_point_labels.clear();
                    result->success = false;
                    result->message = std::string("Feature extraction error: ") + full_err;
                }
                return;
            } catch (...) {
                const std::string full_err = detail_hybrid::describeCurrentExceptionFull();
                RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                    "[SEMANTIC][Hybrid] Feature extraction UNKNOWN_EXCEPTION: %s", full_err.c_str());
                if (result != nullptr) {
                    result->per_point_labels.clear();
                    result->success = false;
                    result->message = std::string("Feature extraction unknown error: ") + full_err;
                }
                return;
            }

            torch::Tensor logits;
            try {
                auto tfeat = torch::from_blob(feat.data(), {static_cast<long>(M), static_cast<long>(feat_dim_)}, torch::kFloat32)
                                 .clone()
                                 .to(infer_device);
                std::shared_lock<std::shared_mutex> lock(st.classifier_mu);
                if (st.validated_num_classes == 0 || st.validated_feat_dim == 0) {
                    throw std::runtime_error("Classifier state not initialized");
                }
                logits = st.classifier.forward({tfeat}).toTensor().to(torch::kCPU);
            } catch (const std::exception& e) {
                const std::string full_err = detail_hybrid::describeExceptionFull(e);
                RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                    "[SEMANTIC][Hybrid] Classifier inference failed: %s", e.what());
                RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                    "[SEMANTIC][Hybrid] Classifier inference FULL_EXCEPTION: %s", full_err.c_str());
                if (result != nullptr) {
                    result->per_point_labels.clear();
                    result->success = false;
                    result->message = std::string("Inference error: ") + full_err;
                }
                return;
            } catch (...) {
                const std::string full_err = detail_hybrid::describeCurrentExceptionFull();
                RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                    "[SEMANTIC][Hybrid] Classifier inference UNKNOWN_EXCEPTION: %s", full_err.c_str());
                if (result != nullptr) {
                    result->per_point_labels.clear();
                    result->success = false;
                    result->message = std::string("Inference unknown error: ") + full_err;
                }
                return;
            }

            if (logits.dim() != 2 || logits.size(0) != static_cast<int64_t>(M)) {
                throw std::runtime_error(
                    "hybrid classifier output shape mismatch: rows=" + std::to_string(logits.size(0)) +
                    " expected_M=" + std::to_string(M));
            }
            num_classes_runtime = static_cast<int>(logits.size(1));
            if (num_classes_runtime <= 0) {
                throw std::runtime_error("hybrid classifier num_classes invalid");
            }
            if (vi == 0) {
                acc_logits.assign(static_cast<size_t>(M) * static_cast<size_t>(num_classes_runtime), 0.0f);
            } else if (static_cast<int>(logits.size(1)) != num_classes_runtime) {
                throw std::runtime_error("hybrid classifier num_classes changed between TTA votes");
            }

            const auto* lp = logits.contiguous().data_ptr<float>();
            for (size_t j = 0; j < acc_logits.size(); ++j) {
                acc_logits[j] += lp[j];
            }
        }

        auto acc_tensor =
            torch::from_blob(acc_logits.data(), {static_cast<long>(M), static_cast<long>(num_classes_runtime)}, torch::kFloat32).clone();
        auto pred = acc_tensor.argmax(1).contiguous();

        std::vector<int64_t> raw_labels(N, 0);
        const int64_t* pred_ptr = pred.data_ptr<int64_t>();
        for (size_t k = 0; k < M; ++k) {
            raw_labels[orig_idx[k]] = pred_ptr[k];
        }

        // 2. Keep raw argmax labels for upstream parity.
        const std::vector<int64_t>& smoothed_labels = raw_labels;
        std::unordered_map<int64_t, size_t> class_hist;
        class_hist.reserve(64);
        for (const auto cls : smoothed_labels) {
            ++class_hist[cls];
        }

        mask = cv::Mat(cfg_.img_h, cfg_.img_w, CV_8U, cv::Scalar(0));
        for (size_t i = 0; i < cloud->size(); ++i) {
            const auto& p = cloud->points[i];
            // 🏛️ [逻辑加固] 使用 st.nn_fov_* 确保 Mask 投影与 Backbone 特征提取完全对齐
            const auto proj = detail_hybrid::projectPoint(p, cfg_.img_w, cfg_.img_h, st.nn_fov_up_rad, st.nn_fov_down_rad);
            if (!proj.valid) continue;
            const int64_t cls = smoothed_labels[i];
            if (cls < 0 || cls > 255) continue;
            mask.at<uint8_t>(proj.y, proj.x) = static_cast<uint8_t>(cls);
        }
        const int nonzero_pixels = cv::countNonZero(mask);
        const int tree_label = cfg_.tree_class_id >= 0 ? cfg_.tree_class_id : 255;
        const int tree_label_u8 = std::clamp(tree_label, 0, 255);
        const int mask_tree_pixels = cv::countNonZero(mask == static_cast<uint8_t>(tree_label_u8));
        const size_t point_tree_count =
            class_hist.count(static_cast<int64_t>(tree_label_u8)) > 0
                ? class_hist[static_cast<int64_t>(tree_label_u8)]
                : 0;
        std::vector<std::pair<int64_t, size_t>> sorted_hist(class_hist.begin(), class_hist.end());
        std::sort(sorted_hist.begin(), sorted_hist.end(), [](const auto& a, const auto& b) {
            return a.second > b.second;
        });
        std::ostringstream topk;
        const size_t topk_n = std::min<size_t>(sorted_hist.size(), 8);
        for (size_t i = 0; i < topk_n; ++i) {
            if (i > 0) topk << ",";
            topk << sorted_hist[i].first << ":" << sorted_hist[i].second;
        }
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][Hybrid][STAGE_RESULT] stage=run cloud_pts=%zu infer_pts=%zu num_vote=%d feat_floats=%zu pred_numel=%ld "
            "mask_nonzero=%d "
            "mask_tree_px=%d tree_label=%d point_tree_count=%zu classes=%zu topk=[%s] mask_wh=%dx%d device=%s",
            N,
            M,
            num_votes,
            static_cast<size_t>(M) * static_cast<size_t>(feat_dim_),
            static_cast<long>(N),
            nonzero_pixels,
            mask_tree_pixels,
            tree_label_u8,
            point_tree_count,
            class_hist.size(),
            topk.str().c_str(),
            mask.cols,
            mask.rows,
            infer_device.str().c_str());

        if (result != nullptr) {
            const auto t1 = std::chrono::steady_clock::now();
            result->per_point_labels.resize(N);
            for (size_t i = 0; i < N; ++i) {
                const int64_t cls = smoothed_labels[i];
                result->per_point_labels[i] = static_cast<uint8_t>(std::clamp(cls, static_cast<int64_t>(0), static_cast<int64_t>(255)));
            }
            result->success = true;
            result->backend_name = name();
            result->message = std::string("device=") + infer_device.str();
            result->inference_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        }
    }

    void maskCloud(const CloudXYZIConstPtr& cloud, const cv::Mat& mask, CloudXYZIPtr& out_cloud, int tree_label,
                   bool dense_for_clustering) override {
        out_cloud.reset(new CloudXYZI());
        if (!cloud || cloud->empty()) return;

        const int label = tree_label >= 0 ? tree_label : 255;
        
        // 🛠️ [彻底修复] 无论外部配置如何，必须保持 Organized 契约 (img_w * img_h)
        // 否则下游 Trellis 算子会因 at(u,v) 2D 索引访问而抛出异常并崩溃。
        out_cloud->width = static_cast<uint32_t>(cfg_.img_w);
        out_cloud->height = static_cast<uint32_t>(cfg_.img_h);
        out_cloud->is_dense = false;
        out_cloud->points.assign(static_cast<size_t>(cfg_.img_w * cfg_.img_h),
                                 pcl::PointXYZI(std::numeric_limits<float>::quiet_NaN(),
                                                std::numeric_limits<float>::quiet_NaN(),
                                                std::numeric_limits<float>::quiet_NaN(),
                                                0.0f));

        size_t proj_valid = 0;
        size_t label_hit = 0;
        size_t valid_tree_points = 0;
        for (const auto& p : cloud->points) {
            // 🏛️ [精度对齐] 使用 st.nn_fov_* 确保 Mask 投影与 LSK3DNet 特征提取位置 100% 重合。
            const auto proj = detail_hybrid::projectPoint(p, cfg_.img_w, cfg_.img_h, fov_up_rad_, fov_down_rad_);
            if (!proj.valid) continue;
            ++proj_valid;
            if (mask.at<uint8_t>(proj.y, proj.x) != static_cast<uint8_t>(label)) continue;
            ++label_hit;
            
            // 写入对应索引位置，保留组织化布局
            const size_t idx = static_cast<size_t>(proj.y * cfg_.img_w + proj.x);
            const auto& prev = out_cloud->points[idx];
            if (!std::isfinite(prev.x) || !std::isfinite(prev.y) || !std::isfinite(prev.z)) {
                ++valid_tree_points;
            }
            out_cloud->points[idx] = p;
        }

        static std::atomic<uint64_t> maskcloud_calls{0};
        const uint64_t call_idx = maskcloud_calls.fetch_add(1, std::memory_order_relaxed) + 1;
        if (call_idx <= 10 || (call_idx % 20 == 0)) {
            const double proj_valid_ratio = cloud->empty()
                ? 0.0
                : (100.0 * static_cast<double>(proj_valid) / static_cast<double>(cloud->size()));
            const double label_hit_ratio = proj_valid == 0
                ? 0.0
                : (100.0 * static_cast<double>(label_hit) / static_cast<double>(proj_valid));
            const double valid_tree_ratio = static_cast<double>(cfg_.img_w * cfg_.img_h) > 0.0
                ? (100.0 * static_cast<double>(valid_tree_points) /
                   static_cast<double>(static_cast<size_t>(cfg_.img_w) * static_cast<size_t>(cfg_.img_h)))
                : 0.0;
            const size_t placeholders = static_cast<size_t>(cfg_.img_w) * static_cast<size_t>(cfg_.img_h) - valid_tree_points;
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                        "[SEMANTIC][HybridMaskCloud][DIAG] call=%lu cloud_pts=%zu proj_valid=%zu(%.2f%%) label_hit=%zu(%.2f%%) "
                        "valid_tree_points=%zu(%.3f%% of image) placeholders=%zu tree_label=%d",
                        static_cast<unsigned long>(call_idx),
                        cloud->size(),
                        proj_valid,
                        proj_valid_ratio,
                        label_hit,
                        label_hit_ratio,
                        valid_tree_points,
                        valid_tree_ratio,
                        placeholders,
                        label);
        }
    }

private:
    SegmentorConfig cfg_;
    double fov_up_rad_ = 0.0;
    double fov_down_rad_ = 0.0;
    int input_dims_ = 0;
    int feat_dim_ = 0;
};

#endif  // USE_TORCH

std::unique_ptr<ISemanticSegmentor> CreateLsk3dnetHybridSemanticSegmentor(const SegmentorConfig& cfg) {
#ifndef USE_TORCH
    (void)cfg;
    throw std::runtime_error("lsk3dnet_hybrid requires USE_TORCH=1 (LibTorch for classifier head)");
#else
    return std::make_unique<Lsk3dnetHybridSemanticSegmentor>(cfg);
#endif
}

}  // namespace automap_pro::v3
