#include "automap_pro/v3/semantic_segmentor.h"

#include <algorithm>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
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

#ifdef USE_TORCH
namespace detail_hybrid {

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
    std::string checkpoint_sha256_hex;
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
        if (j.contains("checkpoint_sha256_hex")) {
            out.checkpoint_sha256_hex = j.at("checkpoint_sha256_hex").get<std::string>();
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
            ::setenv("PYTHONPATH", cfg_.lsk3dnet_repo_root.c_str(), 1);
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
    
    // 🏛️ [逻辑加固] 统一 FOV 状态，解决 C++/Python 映射不一致问题
    double nn_fov_up_rad = 0.0;
    double nn_fov_down_rad = 0.0;
};

HybridSharedState& hybridState() {
    static HybridSharedState s;
    return s;
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
            return;
        }
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
    }

    // 2. 初始化 LibTorch 分类头 (独占锁)
    {
        std::unique_lock<std::shared_mutex> lock(st.classifier_mu);
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
        
        RCLCPP_INFO(rclcpp::get_logger("automap_system"), 
                    "[SEMANTIC][LSK3DNET_HYBRID] Classifier contract validation OK (feat_dim=%u, num_classes=%u, device=%s)",
                    st.validated_feat_dim, st.validated_num_classes, st.device.str().c_str());
    }
}

void inferFeaturesWithRecover(const std::vector<float>& pts, std::vector<float>* feat, torch::Device* out_dev) {
    HybridSharedState& st = hybridState();
    
    // 🏛️ [架构加固] 仅在骨干推理时持有 backbone_mu，允许分类头并发
    std::lock_guard<std::mutex> lock(st.backbone_mu);
    auto run_infer = [&] { st.worker->infer_features(pts, feat); };
    try {
        run_infer();
    } catch (const std::runtime_error& e) {
        const std::string msg = e.what();
        if (msg.find("EOF") != std::string::npos || msg.find("worker") != std::string::npos) {
            st.worker.reset();
            const ClassifierMetaFile meta = loadClassifierMeta(st.frozen_cfg.lsk3dnet_classifier_torchscript);
            st.worker = std::make_unique<HybridPythonWorker>(st.frozen_cfg, meta.checkpoint_sha256_hex);
            run_infer();
        } else {
            throw;
        }
    }
    
    // 获取当前设备状态 (读锁)
    {
        std::shared_lock<std::shared_mutex> lock(st.classifier_mu);
        *out_dev = st.device;
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
                result->success = true;
                result->backend_name = name();
                result->message = "empty cloud";
                result->inference_ms = 0.0;
            }
            return;
        }
        const auto t0 = std::chrono::steady_clock::now();

        const int c = input_dims_;
        std::vector<float> pts(static_cast<size_t>(cloud->size()) * static_cast<size_t>(c), 0.0f);
        for (size_t i = 0; i < cloud->size(); ++i) {
            const auto& p = cloud->points[i];
            const size_t base = i * static_cast<size_t>(c);
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
                pts[base + 0] = 0.0f; pts[base + 1] = 0.0f; pts[base + 2] = 0.0f;
            } else {
                pts[base + 0] = p.x;
                pts[base + 1] = p.y;
                pts[base + 2] = p.z;
            }
            if (c > 3) pts[base + 3] = std::isfinite(p.intensity) ? p.intensity : 0.0f;
        }

        std::vector<float> feat;
        torch::Device infer_device{torch::kCPU};
        try {
            detail_hybrid::inferFeaturesWithRecover(pts, &feat, &infer_device);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Hybrid] Feature extraction failed: %s", e.what());
            if (result != nullptr) {
                result->success = false;
                result->message = std::string("Feature extraction error: ") + e.what();
            }
            return;
        }

        torch::NoGradGuard no_grad;
        torch::Tensor logits;
        try {
            auto tfeat = torch::from_blob(feat.data(), {static_cast<long>(cloud->size()), static_cast<long>(feat_dim_)}, torch::kFloat32)
                             .clone()
                             .to(infer_device);
            detail_hybrid::HybridSharedState& st = detail_hybrid::hybridState();
            
            // 🏛️ [架构演进] 读锁保护分类头推理，支持跨线程并发
            std::shared_lock<std::shared_mutex> lock(st.classifier_mu);
            if (st.validated_num_classes == 0 || st.validated_feat_dim == 0) {
                throw std::runtime_error("Classifier state not initialized");
            }
            logits = st.classifier.forward({tfeat}).toTensor().to(torch::kCPU);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Hybrid] Classifier inference failed: %s", e.what());
            if (result != nullptr) {
                result->success = false;
                result->message = std::string("Inference error: ") + e.what();
            }
            return;
        }
        auto pred = logits.argmax(1).contiguous();
        
        // ... (smoothing logic omitted for brevity, keeping existing) ...
        // 1. Point-level raw labels
        std::vector<int64_t> raw_labels(cloud->size());
        const int64_t* pred_ptr = pred.data_ptr<int64_t>();
        std::copy(pred_ptr, pred_ptr + cloud->size(), raw_labels.begin());

        // 2. Spatial smoothing (Radius-based voting in a sparse grid)
        std::vector<int64_t> smoothed_labels = raw_labels;
        if (cloud->size() > 0) {
            const float filter_radius = 0.5f;
            const int min_neighbors = 3;
            struct VoxelKey {
                int x, y, z;
                bool operator==(const VoxelKey& o) const { return x == o.x && y == o.y && z == o.z; }
            };
            auto hash_fn = [](const VoxelKey& k) {
                return std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1) ^ (std::hash<int>()(k.z) << 2);
            };
            std::unordered_map<VoxelKey, std::vector<size_t>, decltype(hash_fn)> grid(cloud->size() / 2, hash_fn);
            float inv_res = 1.0f / filter_radius;
            for (size_t i = 0; i < cloud->size(); ++i) {
                const auto& p = cloud->points[i];
                grid[{static_cast<int>(std::floor(p.x * inv_res)),
                      static_cast<int>(std::floor(p.y * inv_res)),
                      static_cast<int>(std::floor(p.z * inv_res))}].push_back(i);
            }
            #pragma omp parallel for
            for (size_t i = 0; i < cloud->size(); ++i) {
                const auto& p = cloud->points[i];
                const int64_t my_label = raw_labels[i];
                if (my_label == 0) continue;
                const VoxelKey key{static_cast<int>(std::floor(p.x * inv_res)),
                                   static_cast<int>(std::floor(p.y * inv_res)),
                                   static_cast<int>(std::floor(p.z * inv_res))};
                int same_label_count = 0; int total_neighbors = 0;
                for (int dx = -1; dx <= 1; ++dx) {
                    for (int dy = -1; dy <= 1; ++dy) {
                        for (int dz = -1; dz <= 1; ++dz) {
                            auto it = grid.find({key.x + dx, key.y + dy, key.z + dz});
                            if (it != grid.end()) {
                                for (size_t n_idx : it->second) {
                                    const auto& np = cloud->points[n_idx];
                                    if (((p.x-np.x)*(p.x-np.x) + (p.y-np.y)*(p.y-np.y) + (p.z-np.z)*(p.z-np.z)) < filter_radius * filter_radius) {
                                        total_neighbors++;
                                        if (raw_labels[n_idx] == my_label) same_label_count++;
                                    }
                                }
                            }
                        }
                    }
                }
                if (same_label_count < min_neighbors || static_cast<float>(same_label_count) / std::max(1, total_neighbors) < 0.3f) smoothed_labels[i] = 0;
            }
        }

        mask = cv::Mat(cfg_.img_h, cfg_.img_w, CV_8U, cv::Scalar(0));
        detail_hybrid::HybridSharedState& st = detail_hybrid::hybridState();
        for (size_t i = 0; i < cloud->size(); ++i) {
            const auto& p = cloud->points[i];
            // 🏛️ [逻辑加固] 使用 st.nn_fov_* 确保 Mask 投影与 Backbone 特征提取完全对齐
            const auto proj = detail_hybrid::projectPoint(p, cfg_.img_w, cfg_.img_h, st.nn_fov_up_rad, st.nn_fov_down_rad);
            if (!proj.valid) continue;
            const int64_t cls = smoothed_labels[i];
            if (cls < 0 || cls > 255) continue;
            mask.at<uint8_t>(proj.y, proj.x) = static_cast<uint8_t>(cls);
        }

        if (result != nullptr) {
            const auto t1 = std::chrono::steady_clock::now();
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
        if (dense_for_clustering) {
            out_cloud->width = static_cast<uint32_t>(cfg_.img_w);
            out_cloud->height = static_cast<uint32_t>(cfg_.img_h);
            out_cloud->is_dense = false;
            out_cloud->points.assign(static_cast<size_t>(cfg_.img_w * cfg_.img_h),
                                     pcl::PointXYZI(std::numeric_limits<float>::quiet_NaN(),
                                                    std::numeric_limits<float>::quiet_NaN(),
                                                    std::numeric_limits<float>::quiet_NaN(),
                                                    std::numeric_limits<float>::quiet_NaN()));
        } else {
            out_cloud->height = 1;
            out_cloud->is_dense = false;
            out_cloud->points.reserve(cloud->size());
        }

        for (const auto& p : cloud->points) {
            const auto proj = detail_hybrid::projectPoint(p, cfg_.img_w, cfg_.img_h, fov_up_rad_, fov_down_rad_);
            if (!proj.valid) continue;
            if (mask.at<uint8_t>(proj.y, proj.x) != static_cast<uint8_t>(label)) continue;
            if (dense_for_clustering) {
                out_cloud->points[static_cast<size_t>(proj.y * cfg_.img_w + proj.x)] = p;
            } else {
                out_cloud->points.push_back(p);
            }
        }

        if (!dense_for_clustering) {
            out_cloud->width = static_cast<uint32_t>(out_cloud->points.size());
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
