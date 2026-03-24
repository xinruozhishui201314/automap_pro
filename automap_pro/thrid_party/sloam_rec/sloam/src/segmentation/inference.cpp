#include "inference.h"
#include <boost/make_shared.hpp>
#include <array>
#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <unistd.h>

namespace {
constexpr size_t kCrashRingSize = 24;
constexpr size_t kCrashLineMax = 512;
std::array<std::array<char, kCrashLineMax>, kCrashRingSize> g_crash_ring{};
std::atomic<uint64_t> g_crash_ring_seq{0};
std::atomic<int> g_crash_handler_installed{0};
std::atomic<int> g_crash_dumping{0};

void dumpRecentRunSnapshots(const char* reason) {
  const uint64_t seq = g_crash_ring_seq.load(std::memory_order_acquire);
  char header[192];
  const int n = std::snprintf(header, sizeof(header),
                              "[Segmentation][CrashDump] reason=%s total_samples=%llu ring_size=%zu\n",
                              reason, static_cast<unsigned long long>(seq), kCrashRingSize);
  if (n > 0) {
    ::write(STDERR_FILENO, header, static_cast<size_t>(n));
  }
  if (seq == 0) {
    return;
  }
  const uint64_t begin = (seq > kCrashRingSize) ? (seq - kCrashRingSize) : 0;
  for (uint64_t s = begin; s < seq; ++s) {
    const size_t idx = static_cast<size_t>(s % kCrashRingSize);
    const char* line = g_crash_ring[idx].data();
    const size_t len = ::strnlen(line, kCrashLineMax);
    if (len > 0) {
      ::write(STDERR_FILENO, line, len);
      ::write(STDERR_FILENO, "\n", 1);
    }
  }
}

void segmentationCrashSignalHandler(int sig) {
  if (g_crash_dumping.exchange(1, std::memory_order_acq_rel) == 0) {
    if (sig == SIGABRT) {
      dumpRecentRunSnapshots("SIGABRT");
    } else if (sig == SIGSEGV) {
      dumpRecentRunSnapshots("SIGSEGV");
    } else {
      dumpRecentRunSnapshots("SIGNAL");
    }
  }
  std::signal(sig, SIG_DFL);
  std::raise(sig);
}

void installSegmentationCrashHandlerOnce() {
  int expected = 0;
  if (!g_crash_handler_installed.compare_exchange_strong(expected, 1, std::memory_order_acq_rel)) {
    return;
  }
  std::signal(SIGABRT, segmentationCrashSignalHandler);
  std::signal(SIGSEGV, segmentationCrashSignalHandler);
}

void pushRunSnapshotToCrashRing(uint64_t run_id, size_t cloud_points,
                                double projection_ms, double tensor_ms, double run_ms, double mask_ms,
                                size_t invalid_points, size_t oob_writes, size_t invalid_pixels) {
  const uint64_t seq = g_crash_ring_seq.fetch_add(1, std::memory_order_acq_rel);
  const size_t idx = static_cast<size_t>(seq % kCrashRingSize);
  std::snprintf(
      g_crash_ring[idx].data(), kCrashLineMax,
      "[Segmentation][CrashDump][RunSnapshot] run_id=%llu pts=%zu ms(total=%.3f,proj=%.3f,tensor=%.3f,run=%.3f,mask=%.3f) invalid(points=%zu,oob=%zu,pixels=%zu)",
      static_cast<unsigned long long>(run_id), cloud_points,
      projection_ms + tensor_ms + run_ms + mask_ms, projection_ms, tensor_ms, run_ms, mask_ms,
      invalid_points, oob_writes, invalid_pixels);
}

void pushRunStageToCrashRing(uint64_t run_id, const char* stage, size_t cloud_points,
                             size_t invalid_points, size_t oob_writes, size_t invalid_pixels) {
  const uint64_t seq = g_crash_ring_seq.fetch_add(1, std::memory_order_acq_rel);
  const size_t idx = static_cast<size_t>(seq % kCrashRingSize);
  std::snprintf(
      g_crash_ring[idx].data(), kCrashLineMax,
      "[Segmentation][CrashDump][Stage] run_id=%llu stage=%s pts=%zu invalid(points=%zu,oob=%zu,pixels=%zu)",
      static_cast<unsigned long long>(run_id), stage ? stage : "unknown", cloud_points,
      invalid_points, oob_writes, invalid_pixels);
}

int envIntOr(const char* name, int fallback) {
  const char* v = std::getenv(name);
  if (!v || *v == '\0') return fallback;
  char* end = nullptr;
  long parsed = std::strtol(v, &end, 10);
  if (end == v) return fallback;
  if (parsed < 1) return fallback;
  if (parsed > 1024) return 1024;
  return static_cast<int>(parsed);
}

double nowMs() {
  using clock = std::chrono::steady_clock;
  return std::chrono::duration<double, std::milli>(clock::now().time_since_epoch()).count();
}

double percentile(std::vector<double> values, double q) {
  if (values.empty()) {
    return 0.0;
  }
  if (q < 0.0) q = 0.0;
  if (q > 1.0) q = 1.0;
  const size_t k = static_cast<size_t>(q * static_cast<double>(values.size() - 1));
  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(k), values.end());
  return values[k];
}

std::vector<int64_t> normalizeDims(const std::vector<int64_t>& dims,
                                   const std::vector<int64_t>& fallback,
                                   const std::string& tag) {
    std::vector<int64_t> normalized = dims;
    for (size_t i = 0; i < normalized.size(); ++i) {
        if (normalized[i] <= 0) {
            const int64_t replacement = (i < fallback.size() && fallback[i] > 0) ? fallback[i] : 1;
            std::cout << "[Segmentation] " << tag << " dim[" << i << "]=" << normalized[i]
                      << " is dynamic/invalid, use " << replacement << std::endl;
            normalized[i] = replacement;
        }
    }
    return normalized;
}

size_t checkedProduct(const std::vector<int64_t>& dims, const std::string& tag) {
    size_t total = 1;
    for (const int64_t d : dims) {
        if (d <= 0) {
            throw std::runtime_error("[Segmentation] invalid non-positive " + tag + " dimension");
        }
        const size_t du = static_cast<size_t>(d);
        if (total > std::numeric_limits<size_t>::max() / du) {
            throw std::runtime_error("[Segmentation] " + tag + " tensor size overflow");
        }
        total *= du;
    }
    return total;
}
}  // namespace


namespace seg {
Segmentation::Segmentation(const std::string modelFilePath,
    const float fov_up, const float fov_down, const int img_w, const int img_h,
    const int input_channels, const int num_classes, const int tree_class_id,
    const std::vector<float>& input_mean, const std::vector<float>& input_std, bool do_destagger) {
  _fov_up = fov_up / 180.0 * M_PI;    // field of view up in radians
  _fov_down = fov_down / 180.0 * M_PI;  // field of view down in radians
  _fov = std::abs(_fov_down) + std::abs(_fov_up); // get field of view total in radians
  _img_w = img_w;
  _img_h = img_h;
  _img_d = input_channels;
  _num_classes = num_classes;
  _tree_class_id = tree_class_id;
  _user_input_mean = input_mean;
  _user_input_std = input_std;
  _verbose = false;
  _do_destagger = do_destagger;
  installSegmentationCrashHandlerOnce();

  const std::string sessionName = "SLOAMSeg";
  // specify number of CPU threads allowed for semantic segmentation inference to use
  _startONNXSession(sessionName, modelFilePath, false, 3);
}

void Segmentation::_startONNXSession(const std::string sessionName, const std::string modelFilePath, bool useCUDA, size_t numThreads){
    Ort::SessionOptions sessionOptions;
    // Default to single-thread for maximum stability; can be overridden by env.
    const int intra_threads = envIntOr("AUTOMAP_ORT_INTRA_OP_THREADS", 1);
    const int inter_threads = envIntOr("AUTOMAP_ORT_INTER_OP_THREADS", 1);
    const int safe_mode = envIntOr("AUTOMAP_ORT_SAFE_MODE", 1);
    sessionOptions.SetIntraOpNumThreads(intra_threads);
    sessionOptions.SetInterOpNumThreads(inter_threads);
    // Sets graph optimization level
    // Available levels are
    // ORT_DISABLE_ALL -> To disable all optimizations
    // ORT_ENABLE_BASIC -> To enable basic optimizations (Such as redundant node
    // removals) ORT_ENABLE_EXTENDED -> To enable extended optimizations
    // (Includes level 1 + more complex optimizations like node fusions)
    // ORT_ENABLE_ALL -> To Enable All possible optimizations
    // Prefer stability over aggressive fusion: EXTENDED may produce FusedConv kernels
    // that are unstable in some ORT build/runtime combinations.
    sessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_BASIC);
    if (safe_mode == 1) {
        // Productization hardening: avoid ORT internal parallel execution race surfaces.
        sessionOptions.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
    }
    if (useCUDA){
        // Using CUDA backend
        // https://github.com/microsoft/onnxruntime/blob/v1.8.2/include/onnxruntime/core/session/onnxruntime_cxx_api.h#L329
        OrtCUDAProviderOptions cuda_options{0};
        sessionOptions.AppendExecutionProvider_CUDA(cuda_options);
    }

    _env = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_ERROR, sessionName.c_str());

    _session = std::make_unique<Ort::Session>(*_env, modelFilePath.c_str(), sessionOptions);
    std::cout << "[Segmentation] ORT Session options: safe_mode=" << safe_mode
              << " intra_threads=" << intra_threads
              << " inter_threads=" << inter_threads
              << " graph_opt=BASIC" << std::endl;

    _memoryInfo = std::make_unique<Ort::MemoryInfo>(Ort::MemoryInfo::CreateCpu(
        OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault));

    // Ort::AllocatorWithDefaultOptions allocator;
    // INPUT 
    char* inputName = _session->GetInputName(0, _allocator);
    _inputNamesStr = {std::string(inputName)};
    _inputNames = {_inputNamesStr[0].c_str()};
    _allocator.Free(inputName); // Free the C-style name string after copying
    std::cout << "[Segmentation] Input Name: " << _inputNames[0] << std::endl;

    Ort::TypeInfo inputTypeInfo = _session->GetInputTypeInfo(0);
    auto inputTensorInfo = inputTypeInfo.GetTensorTypeAndShapeInfo();
    _inputDims = inputTensorInfo.GetShape();
    // _inputDims = {1, 64, 2048, 2};
    std::cout << "[Segmentation] Input Dimensions: "; printVector(_inputDims);

    // OUTPUT 
    char* outputName = _session->GetOutputName(0, _allocator);
    _outputNamesStr = {std::string(outputName)};
    _outputNames = {_outputNamesStr[0].c_str()};
    _allocator.Free(outputName); // Free the C-style name string after copying
    std::cout << "[Segmentation] Output Name: " << _outputNames[0] << std::endl;

    Ort::TypeInfo outputTypeInfo = _session->GetOutputTypeInfo(0);
    auto outputTensorInfo = outputTypeInfo.GetTensorTypeAndShapeInfo();
    // ONNXTensorElementDataType outputType = outputTensorInfo.GetElementType();

    _outputDims = outputTensorInfo.GetShape();
    // _outputDims = {1, 64, 2048, 2};
    std::cout << "[Segmentation] Output Dimensions: "; printVector(_outputDims);

    _inputDims = normalizeDims(_inputDims, {1, (_img_d > 0 ? _img_d : 1), _img_h, _img_w}, "input");
    _outputDims = normalizeDims(_outputDims, {1, (_num_classes > 0 ? _num_classes : 1), _img_h, _img_w}, "output");

    if (_inputDims.size() < 4 || _outputDims.size() < 4) {
        throw std::runtime_error("[Segmentation] unsupported model rank: expect at least 4D NCHW");
    }
    const int model_input_channels = static_cast<int>(_inputDims[1]);
    const int model_num_classes = static_cast<int>(_outputDims[1]);
    const int model_input_h = static_cast<int>(_inputDims[2]);
    const int model_input_w = static_cast<int>(_inputDims[3]);
    const int model_output_h = static_cast<int>(_outputDims[2]);
    const int model_output_w = static_cast<int>(_outputDims[3]);
    if (_img_d > 0 && _img_d != model_input_channels) {
        throw std::runtime_error("[Segmentation] semantic.input_channels mismatches model input channels");
    }
    if (_num_classes > 0 && _num_classes != model_num_classes) {
        throw std::runtime_error("[Segmentation] semantic.num_classes mismatches model output classes");
    }
    if (model_input_h != _img_h || model_input_w != _img_w) {
        throw std::runtime_error(
            "[Segmentation] model input H/W mismatches semantic.img_h/img_w (would corrupt tensor layout)");
    }
    if (model_output_h != _img_h || model_output_w != _img_w) {
        throw std::runtime_error(
            "[Segmentation] model output H/W mismatches semantic.img_h/img_w (would corrupt mask layout)");
    }
    _img_d = model_input_channels;
    _num_classes = model_num_classes;
    if (_tree_class_id < 0 || _tree_class_id >= _num_classes) {
        _tree_class_id = (_num_classes > 2) ? 2 : 1;
        std::cout << "[Segmentation] tree_class_id auto-fallback to " << _tree_class_id << std::endl;
    }

    std::cout << "[Segmentation] Normalized Input Dimensions: "; printVector(_inputDims);
    std::cout << "[Segmentation] Normalized Output Dimensions: "; printVector(_outputDims);
    std::cout << "[Segmentation] Effective config: input_channels=" << _img_d
              << " num_classes=" << _num_classes
              << " tree_class_id=" << _tree_class_id << std::endl;

    _finalizeInputNormStats();

    // _inputTensorSize = _img_w * _img_h * _img_d;
    // _outputTensorSize = _img_w * _img_h * _img_d;
    _inputTensorSize = checkedProduct(_inputDims, "input");
    _outputTensorSize = checkedProduct(_outputDims, "output");
    _input_tensor_buffer.resize(_inputTensorSize);
    _invalid_idx_buffer.clear();
    _invalid_idx_buffer.reserve(static_cast<size_t>(_img_w) * static_cast<size_t>(_img_h));
}

void Segmentation::_finalizeInputNormStats() {
    if (!_user_input_mean.empty() && static_cast<int>(_user_input_mean.size()) != _img_d) {
        throw std::runtime_error("[Segmentation] semantic.input_mean length must equal model input channel count");
    }
    if (!_user_input_std.empty() && static_cast<int>(_user_input_std.size()) != _img_d) {
        throw std::runtime_error("[Segmentation] semantic.input_std length must equal model input channel count");
    }
    _norm_mean.resize(static_cast<size_t>(_img_d));
    _norm_std.resize(static_cast<size_t>(_img_d));
    for (int i = 0; i < _img_d; ++i) {
        const float def_mean = (i == 0) ? 12.97f : 0.0f;
        const float def_std = (i == 0) ? 12.35f : 1.0f;
        _norm_mean[static_cast<size_t>(i)] =
            !_user_input_mean.empty() ? _user_input_mean[static_cast<size_t>(i)] : def_mean;
        _norm_std[static_cast<size_t>(i)] =
            !_user_input_std.empty() ? _user_input_std[static_cast<size_t>(i)] : def_std;
    }
    for (size_t i = 0; i < _norm_std.size(); ++i) {
        if (_norm_std[i] < 1e-6f) {
            throw std::runtime_error("[Segmentation] semantic.input_std contains non-positive value (division unsafe)");
        }
    }
    if (!_norm_mean.empty()) {
        std::cout << "[Segmentation] input_norm applied: mean[0]=" << _norm_mean[0]
                  << " std[0]=" << _norm_std[0] << std::endl;
    }
}

std::vector<std::vector<float>> Segmentation::_doProjection(const std::vector<float>& scan, const uint32_t& num_points){
  
  std::vector<float> invalid_input(static_cast<size_t>(_img_d), 0.0f);
  constexpr float kMinRange = 1e-6f;
  size_t invalid_points = 0;
  size_t oob_writes = 0;

  std::vector<float> ranges;
  std::vector<float> xs;
  std::vector<float> ys;
  std::vector<float> zs;
  std::vector<float> intensitys;

  std::vector<float> proj_xs_tmp;
  std::vector<float> proj_ys_tmp;
  ranges.reserve(num_points);
  xs.reserve(num_points);
  ys.reserve(num_points);
  zs.reserve(num_points);
  intensitys.reserve(num_points);
  proj_xs_tmp.reserve(num_points);
  proj_ys_tmp.reserve(num_points);

  for (uint32_t i = 0; i < num_points; i++) {
    float x = scan[4 * i];
    float y = scan[4 * i + 1];
    float z = scan[4 * i + 2];
    float intensity = scan[4 * i + 3];
    const bool finite_xyz = std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && std::isfinite(intensity);
    float range = finite_xyz ? std::sqrt(x*x+y*y+z*z) : 0.0f;
    if (!std::isfinite(range)) {
      range = 0.0f;
    }
    const bool valid_range = std::isfinite(range) && range > kMinRange;
    ranges.push_back(range);
    xs.push_back(x);
    ys.push_back(y);
    zs.push_back(z);
    intensitys.push_back(intensity);

    if (!finite_xyz || !valid_range) {
      // Keep vector sizes aligned with input point count.
      ++invalid_points;
      proj_xs_tmp.push_back(0.0f);
      proj_ys_tmp.push_back(0.0f);
      continue;
    }

    // get angles
    float yaw = -std::atan2(y, x);
    const float sin_pitch = std::clamp(z / range, -1.0f, 1.0f);
    float pitch = std::asin(sin_pitch);

    // get projections in image coords
    float proj_x = 0.5 * (yaw / M_PI + 1.0); // in [0.0, 1.0]
    float proj_y = 1.0 - (pitch + std::abs(_fov_down)) / _fov; // in [0.0, 1.0]

    // scale to image size using angular resolution
    proj_x *= _img_w; // in [0.0, W]
    proj_y *= _img_h; // in [0.0, H]

    // round and clamp for use as index
    proj_x = std::floor(proj_x);
    proj_x = std::min(_img_w - 1.0f, proj_x);
    proj_x = std::max(0.0f, proj_x); // in [0,W-1]
    if (!std::isfinite(proj_x)) proj_x = 0.0f;
    proj_xs_tmp.push_back(proj_x);

    proj_y = std::floor(proj_y);
    proj_y = std::min(_img_h - 1.0f, proj_y);
    proj_y = std::max(0.0f, proj_y); // in [0,H-1]
    if (!std::isfinite(proj_y)) proj_y = 0.0f;
    proj_ys_tmp.push_back(proj_y);
  }

  // stope a copy in original order
  proj_xs = proj_xs_tmp;
  proj_ys = proj_ys_tmp;

  // order in decreasing depth
  std::vector<size_t> orders = sort_indexes(ranges);
  std::vector<float> sorted_proj_xs;
  sorted_proj_xs.reserve(num_points);
  std::vector<float> sorted_proj_ys;
  sorted_proj_ys.reserve(num_points);
  std::vector<std::vector<float>> inputs;
  inputs.reserve(num_points);

  for (size_t idx : orders){
    sorted_proj_xs.push_back(proj_xs[idx]);
    sorted_proj_ys.push_back(proj_ys[idx]);
    std::vector<float> input(static_cast<size_t>(_img_d), 0.0f);
    if (_img_d > 0) input[0] = ranges[idx];
    if (_img_d > 1) input[1] = xs[idx];
    if (_img_d > 2) input[2] = ys[idx];
    if (_img_d > 3) input[3] = zs[idx];
    if (_img_d > 4) input[4] = intensitys[idx];
    inputs.push_back(input);
  }

  // assing to images
  std::vector<std::vector<float>> range_image(_img_w * _img_h);

  // zero initialize
  for (uint32_t i = 0; i < range_image.size(); ++i) {
      range_image[i] = invalid_input;
  }

  for (uint32_t i = 0; i < inputs.size(); ++i) {
    const int proj_idx = static_cast<int>(sorted_proj_ys[i] * _img_w + sorted_proj_xs[i]);
    if (proj_idx < 0 || static_cast<size_t>(proj_idx) >= range_image.size()) {
      ++oob_writes;
      continue;
    }
    range_image[static_cast<size_t>(proj_idx)] = inputs[i];
  }

  _last_projection_invalid_points = invalid_points;
  _last_projection_oob_writes = oob_writes;
  _last_projection_points = static_cast<size_t>(num_points);

  return range_image;
}

void Segmentation::_makeTensor(std::vector<std::vector<float>>& projected_data, std::vector<float>& tensor, std::vector<size_t>& invalid_idxs){
  const int channel_offset = _img_h * _img_w;
  const int num_pixels = static_cast<int>(projected_data.size());
  const int num_channels = _img_d;

  // Cache norm constants locally to reduce indirection
  std::vector<float> inv_std(num_channels);
  for (int i = 0; i < num_channels; ++i) {
      inv_std[i] = 1.0f / (_norm_std[i] > 1e-6f ? _norm_std[i] : 1.0f);
  }
  const float* mean_ptr = _norm_mean.data();
  const float* inv_std_ptr = inv_std.data();

  // Use OpenMP for parallel processing.
  // Since we need to collect invalid_idxs, we use thread-local vectors to avoid frequent locking.
  #pragma omp parallel
  {
      std::vector<size_t> local_invalid_idxs;
      #pragma omp for schedule(static)
      for (int pixel_id = 0; pixel_id < num_pixels; pixel_id++) {
          const auto& pixel_data = projected_data[pixel_id];
          
          // Fast check: if the first channel (typically range) is 0 or NaN, it's invalid.
          // In SLOAM projection, if the first channel is 0, the pixel is unassigned/invalid.
          const float r = pixel_data[0];
          const bool all_zeros = (r == 0.0f || !std::isfinite(r));
          
          if (all_zeros) {
              local_invalid_idxs.push_back(static_cast<size_t>(pixel_id));
              // Ensure tensor is clean for invalid pixels
              for (int i = 0; i < num_channels; i++) {
                  tensor[channel_offset * i + pixel_id] = 0.0f;
              }
          } else {
              for (int i = 0; i < num_channels; i++) {
                  // Normalization: (x - mean) / std  => (x - mean) * (1/std)
                  tensor[channel_offset * i + pixel_id] = (pixel_data[i] - mean_ptr[i]) * inv_std_ptr[i];
              }
          }
      }
      
      #pragma omp critical(segmentation_invalid_idxs)
      {
          invalid_idxs.insert(invalid_idxs.end(), local_invalid_idxs.begin(), local_invalid_idxs.end());
      }
  }
}

void Segmentation::_destaggerCloud(const Cloud::Ptr cloud, Cloud::Ptr& outCloud){
  bool col_valid = true;

  for(auto irow = 0; irow < _img_h; irow++){
    for(auto icol = 0; icol < _img_w; icol++){
      auto im_col = icol;
      // Ouster data needs a shift every other row
      if(irow % 2 == 0){
        im_col += 32;
        if(im_col < 0 || im_col > _img_w){
          col_valid = false;
          im_col = im_col % _img_w;
        }
      }

      if(col_valid){
        const auto& pt = cloud->points[irow * _img_w + icol];
        auto& outPt = outCloud->points[irow * _img_w + im_col];
        // const auto& pt = cloud->at(icol, irow);
        // auto& outPt = outCloud->at(im_col, irow);
        outPt.x = pt.x;
        outPt.y = pt.y;
        outPt.z = pt.z;
      }
      
      col_valid = true;
    }
  }
}

void Segmentation::maskCloud(const Cloud::Ptr cloud,
                              cv::Mat mask,
                              Cloud::Ptr& outCloud,
                              unsigned char val,
                              bool dense) {
  std::lock_guard<std::recursive_mutex> lock(_run_mutex);

  Cloud::Ptr tempCloud(new Cloud);
  const size_t expected_mask_px = static_cast<size_t>(_img_h) * static_cast<size_t>(_img_w);
  if (mask.rows != _img_h || mask.cols != _img_w || mask.type() != CV_8U || !mask.isContinuous()) {
    std::cerr << "[Segmentation][maskCloud][CONTRACT_VIOLATION] "
              << "mask rows=" << mask.rows << " cols=" << mask.cols << " type=" << mask.type()
              << " is_contiguous=" << (mask.isContinuous() ? 1 : 0)
              << " expected_rows=" << _img_h << " expected_cols=" << _img_w
              << " expected_type=" << CV_8U << std::endl;
    throw std::runtime_error("[Segmentation] maskCloud: mask must be CV_8U continuous " +
                             std::to_string(_img_h) + "x" + std::to_string(_img_w));
  }
  const size_t cloud_n = cloud->size();
  if (proj_xs.size() != cloud_n || proj_ys.size() != cloud_n) {
    std::cerr << "[Segmentation][maskCloud][CONTRACT_VIOLATION] "
              << "cloud_n=" << cloud_n << " proj_xs=" << proj_xs.size() << " proj_ys=" << proj_ys.size()
              << " hint=call run() then maskCloud() with the same cloud instance" << std::endl;
    throw std::runtime_error(
        "[Segmentation] maskCloud: projection grid mismatch cloud size (call run() on same cloud first)");
  }
  tempCloud->points.reserve(cloud_n);

  for (size_t i = 0; i < cloud_n; i++) {
    size_t proj_idx = static_cast<size_t>(proj_ys[i]) * static_cast<size_t>(_img_w) +
                      static_cast<size_t>(proj_xs[i]);
    if (proj_idx >= expected_mask_px) {
      continue;
    }
    unsigned char m = mask.data[proj_idx];

    if(m == val){
        tempCloud->points.push_back(cloud->points[i]);
    } else if(dense){
      Point p;
      p.x = p.y = p.z = std::numeric_limits<float>::quiet_NaN();
      tempCloud->points.push_back(p);
    } 
  }

  pcl::copyPointCloud(*tempCloud, *outCloud);
  if(dense){
    // destagger. TODO: Do this with mask    
    // Adapted from Chao's driver
    if (_do_destagger){
      const size_t expected_dense_points = static_cast<size_t>(_img_h) * static_cast<size_t>(_img_w);
      if (tempCloud->size() != expected_dense_points) {
        std::cerr << "[Segmentation][maskCloud][DENSE_LAYOUT_MISMATCH] "
                  << "requested_dense=1 do_destagger=1 temp_cloud_size=" << tempCloud->size()
                  << " expected=" << expected_dense_points
                  << " action=fallback_to_sparse_output" << std::endl;
        outCloud = tempCloud;
        outCloud->width = outCloud->points.size();
        outCloud->height = 1;
        outCloud->is_dense = false;
        return;
      }
      _destaggerCloud(tempCloud, outCloud);
    } else {
       outCloud = tempCloud;
    }
    outCloud->width = _img_w;
    outCloud->height = _img_h;
    outCloud->is_dense = true;
  } else {
    outCloud->width = outCloud->points.size();
    outCloud->height = 1;
    outCloud->is_dense = false;
  }

  // outCloud->header = cloud->header;
}

void Segmentation::_mask(const float* output, const std::vector<size_t>& invalid_idxs, cv::Mat& maskImg){
  const int channel_offset = _img_w * _img_h;
  const int num_classes = _num_classes;
  const int tree_id = _tree_class_id;
  const size_t required_bytes = static_cast<size_t>(channel_offset) * sizeof(unsigned char);
  const size_t actual_bytes = static_cast<size_t>(maskImg.rows) * static_cast<size_t>(maskImg.step);
  
  if (maskImg.type() != CV_8U || !maskImg.isContinuous() || actual_bytes < required_bytes) {
    std::cerr << "[Segmentation][_mask][CONTRACT_VIOLATION] "
              << "mask rows=" << maskImg.rows << " cols=" << maskImg.cols << " type=" << maskImg.type()
              << " is_contiguous=" << (maskImg.isContinuous() ? 1 : 0)
              << " step=" << maskImg.step
              << " actual_bytes=" << actual_bytes
              << " required_bytes=" << required_bytes
              << " img_wh=" << _img_w << "x" << _img_h << std::endl;
    throw std::runtime_error("[Segmentation] _mask target buffer invalid: size/type mismatch");
  }

  unsigned char* mask_ptr = maskImg.data;

  // Optimized parallel argmax over classes
  #pragma omp parallel for schedule(static)
  for (int pixel_id = 0; pixel_id < channel_offset; pixel_id++) {
      float max_val = output[pixel_id]; // First channel probability (typically background)
      int best_class = 0;
      
      for (int c = 1; c < num_classes; c++) {
          float val = output[channel_offset * c + pixel_id];
          if (val > max_val) {
              max_val = val;
              best_class = c;
          }
      }
      
      // Map tree class to 255 for visualization consistency, others to class index
      mask_ptr[pixel_id] = (best_class == tree_id) ? 255 : static_cast<unsigned char>(best_class);
  }

  // Final zero-out of invalid pixels (those with no range or NaN)
  #pragma omp parallel for if(invalid_idxs.size() > 1000)
  for (size_t i = 0; i < invalid_idxs.size(); ++i) {
      mask_ptr[invalid_idxs[i]] = 0;
  }
}

void Segmentation::_preProcessRange(const cv::Mat& img, cv::Mat& pImg, float maxDist) {
  std::vector<cv::Mat> channels(2);
  cv::split(img, channels);

  pImg = img.clone();

  // Normalize
  double minVal = 0;
  double maxVal = 0;
  cv::minMaxIdx(channels[1], &minVal, &maxVal);
  float rangeMultiplier = 1.0 / 100.0;
  float intensityMultipler = 1.0 / (float)maxVal;
  cv::multiply(pImg, cv::Scalar(rangeMultiplier, intensityMultipler),
               pImg);
}

void Segmentation::_argmax(const float *in, cv::Mat& maskImg){
  std::vector<unsigned char> max;
  size_t numClasses_ = 2;
  size_t outSize = _img_h * _img_w * numClasses_;
  for (unsigned int i = 0; i < outSize; i += numClasses_) {
    unsigned int maxIdx = i;
    unsigned char outIdx = 0;
    for (unsigned int c = 1; c < numClasses_; c++) {
      if (in[maxIdx] < in[i + c]) {
        maxIdx = i + c;
        outIdx = c;
      }
    }
    if(outIdx == 1) outIdx = 255;
    max.push_back(outIdx);
  }
  memcpy(maskImg.data, max.data(), max.size()*sizeof(unsigned char));
}

void Segmentation::runERF(cv::Mat& rImg, cv::Mat& maskImg){
    cv::Mat pImg(rImg.rows, rImg.cols, CV_32FC2, cv::Scalar(0));
    _preProcessRange(rImg, pImg, 30);
    
    std::vector<float> outputTensorValues(_outputTensorSize);
    std::vector<float> inputTensorValues(_inputTensorSize);

    auto imgSize = pImg.rows * pImg.cols * pImg.channels(); 
    memcpy(inputTensorValues.data(), pImg.data, imgSize * sizeof(float));
    std::cout << "Tensor size: " << _inputTensorSize << std::endl;
    std::cout << "Tensor size: " << _outputTensorSize << std::endl;
    std::cout << "DAta size: " << imgSize << std::endl;

    std::vector<Ort::Value> inputTensors;
    std::vector<Ort::Value> outputTensors;
    inputTensors.push_back(Ort::Value::CreateTensor<float>(
        *_memoryInfo, inputTensorValues.data(), _inputTensorSize, _inputDims.data(),
        _inputDims.size()));

    outputTensors.push_back(Ort::Value::CreateTensor<float>(
        *_memoryInfo, outputTensorValues.data(), _outputTensorSize,
        _outputDims.data(), _outputDims.size()));

    _session->Run(Ort::RunOptions{nullptr}, _inputNames.data(),
                inputTensors.data(), 1, _outputNames.data(),
                outputTensors.data(), 1);

    float* outData = outputTensors.front().GetTensorMutableData<float>();
    // int dims[] = {3,64,2048};
    // cv::Mat result = cv::Mat(3, dims, CV_32F, outData);
    // cv::FileStorage file("/opt/bags/inf/res.ext", cv::FileStorage::WRITE);
    // Write to file!
    // file << "matName" << result;
    // std::cout << sizeof(outData) << std::endl;
    _argmax(outData, maskImg);
}

void Segmentation::run(const Cloud::Ptr cloud, cv::Mat& maskImg){
    const uint64_t run_id = ++_run_seq;
    std::lock_guard<std::recursive_mutex> lock(_run_mutex);
    if (!cloud) {
      std::cerr << "[Segmentation][run][CONTRACT_VIOLATION] run_id=" << run_id << " cloud=nullptr" << std::endl;
      throw std::runtime_error("[Segmentation] run cloud is null");
    }
    if (cloud->empty()) {
      std::cerr << "[Segmentation][run][CONTRACT_VIOLATION] run_id=" << run_id << " cloud is empty" << std::endl;
      throw std::runtime_error("[Segmentation] run cloud is empty");
    }
    if (maskImg.rows != _img_h || maskImg.cols != _img_w || maskImg.type() != CV_8U || !maskImg.isContinuous()) {
      std::cerr << "[Segmentation][run][CONTRACT_VIOLATION] "
                << "run_id=" << run_id << " "
                << "mask rows=" << maskImg.rows << " cols=" << maskImg.cols << " type=" << maskImg.type()
                << " is_contiguous=" << (maskImg.isContinuous() ? 1 : 0)
                << " expected_rows=" << _img_h << " expected_cols=" << _img_w
                << " expected_type=" << CV_8U << std::endl;
      throw std::runtime_error("[Segmentation] run expects preallocated CV_8U mask with model HxW");
    }
    const size_t cloud_wh = static_cast<size_t>(cloud->width) * static_cast<size_t>(cloud->height);
    if (cloud_wh != cloud->size()) {
      std::cerr << "[Segmentation][run][INPUT_LAYOUT_WARNING] "
                << "run_id=" << run_id << " "
                << "cloud_size=" << cloud->size()
                << " width=" << cloud->width
                << " height=" << cloud->height
                << " width*height=" << cloud_wh
                << " hint=ensure organized cloud metadata is consistent" << std::endl;
    }
    std::cout << "[Segmentation][run][BEGIN] run_id=" << run_id
              << " cloud_size=" << cloud->size()
              << " cloud_wh=" << cloud->width << "x" << cloud->height
              << " mask_wh=" << maskImg.cols << "x" << maskImg.rows
              << " dims(in=" << _img_d << "x" << _img_h << "x" << _img_w
              << ",classes=" << _num_classes << ")" << std::endl;
    pushRunStageToCrashRing(run_id, "BEGIN", cloud->size(), 0, 0, 0);

    std::vector<float> cloudVector;
    cloudVector.reserve(cloud->size() * 4);
    for (const auto& point : cloud->points) {
        cloudVector.push_back(point.x); cloudVector.push_back(point.y);
        cloudVector.push_back(point.z); cloudVector.push_back(point.intensity);
    }

    const double t_proj_begin = nowMs();
    if(_verbose){
      std::cout << "Projecting data" << std::endl;
      _timer.tic();
    }
    // 不信任 width*height 元数据，按真实点数投影，避免越界读取 cloudVector。
    const uint32_t num_points = static_cast<uint32_t>(cloud->size());
    auto netInput = _doProjection(cloudVector, num_points);
    if (_last_projection_invalid_points > 0 || _last_projection_oob_writes > 0) {
      std::cerr << "[Segmentation][run][PROJECTION_DIAG] run_id=" << run_id
                << " points=" << _last_projection_points
                << " invalid_points=" << _last_projection_invalid_points
                << " oob_writes=" << _last_projection_oob_writes
                << " img_wh=" << _img_w << "x" << _img_h << std::endl;
    }
    pushRunStageToCrashRing(run_id, "PROJ_DONE", cloud->size(),
                            _last_projection_invalid_points, _last_projection_oob_writes, 0);

    // int dims[] = {64,2048};
    // std::vector<float> flattened_inp;
    // for (int i = 0; i < _img_h*_img_w; ++i) {
    //   size_t proj_idx = proj_ys[i] * _img_w + proj_xs[i];
    //   flattened_inp.push_back(netInput[proj_idx][0]);
    // }
    // cv::Mat result = cv::Mat(2, dims, CV_32F, flattened_inp.data());
    // cv::FileStorage file("/opt/bags/inf/inp.ext", cv::FileStorage::WRITE);
    // file << "mat" << result;

    if(_verbose){
      _timer.toc();
    }
    const double t_proj_end = nowMs();

    const double t_tensor_begin = nowMs();
    if (_input_tensor_buffer.size() != _inputTensorSize) {
      _input_tensor_buffer.resize(_inputTensorSize);
    }
    _invalid_idx_buffer.clear();

    if(_verbose){
      std::cout << "Making tensor" << std::endl;
      _timer.tic();
    }
    
    _makeTensor(netInput, _input_tensor_buffer, _invalid_idx_buffer);
    if (_invalid_idx_buffer.size() > (static_cast<size_t>(_img_w) * static_cast<size_t>(_img_h) * 95 / 100)) {
      std::cerr << "[Segmentation][run][TENSOR_DIAG] run_id=" << run_id
                << " invalid_pixels=" << _invalid_idx_buffer.size()
                << " total_pixels=" << (static_cast<size_t>(_img_w) * static_cast<size_t>(_img_h))
                << " ratio=" << (100.0 * static_cast<double>(_invalid_idx_buffer.size()) /
                                 static_cast<double>(static_cast<size_t>(_img_w) * static_cast<size_t>(_img_h)))
                << "%" << std::endl;
    }
    pushRunStageToCrashRing(run_id, "TENSOR_DONE", cloud->size(),
                            _last_projection_invalid_points, _last_projection_oob_writes,
                            _invalid_idx_buffer.size());

    if(_verbose){
      _timer.toc();
    }
    const double t_tensor_end = nowMs();

    const double t_run_begin = nowMs();
    if(_verbose){
      std::cout << "Running Inference" << std::endl;
      _timer.tic();
    }
    std::vector<Ort::Value> inputTensors;
    inputTensors.push_back(Ort::Value::CreateTensor<float>(
        *_memoryInfo, _input_tensor_buffer.data(), _inputTensorSize, _inputDims.data(),
        _inputDims.size()));
    pushRunStageToCrashRing(run_id, "RUN_PRE", cloud->size(),
                            _last_projection_invalid_points, _last_projection_oob_writes,
                            _invalid_idx_buffer.size());
    auto outputTensors = _session->Run(Ort::RunOptions{nullptr}, _inputNames.data(),
                                       inputTensors.data(), 1, _outputNames.data(), 1);
    pushRunStageToCrashRing(run_id, "RUN_POST", cloud->size(),
                            _last_projection_invalid_points, _last_projection_oob_writes,
                            _invalid_idx_buffer.size());
    if(_verbose){
      _timer.toc();
    }
    const double t_run_end = nowMs();

    const double t_mask_begin = nowMs();
    if(_verbose){
      std::cout << "Masking" << std::endl;
      _timer.tic();
    }

    float* outData = outputTensors.front().GetTensorMutableData<float>();
    _mask(outData, _invalid_idx_buffer, maskImg);

    if(_verbose){
      _timer.toc();
    }
    const double t_mask_end = nowMs();

    _recordStageTiming(
        t_proj_end - t_proj_begin,
        t_tensor_end - t_tensor_begin,
        t_run_end - t_run_begin,
        t_mask_end - t_mask_begin,
        cloud->size());
    pushRunSnapshotToCrashRing(
        run_id, cloud->size(),
        t_proj_end - t_proj_begin,
        t_tensor_end - t_tensor_begin,
        t_run_end - t_run_begin,
        t_mask_end - t_mask_begin,
        _last_projection_invalid_points,
        _last_projection_oob_writes,
        _invalid_idx_buffer.size());
    const double total_ms = (t_mask_end - t_proj_begin);
    if (total_ms > 150.0) {
      std::cerr << "[Segmentation][run][SLOW_FRAME_DIAG] run_id=" << run_id
                << " total_ms=" << total_ms
                << " projection_ms=" << (t_proj_end - t_proj_begin)
                << " tensor_ms=" << (t_tensor_end - t_tensor_begin)
                << " run_ms=" << (t_run_end - t_run_begin)
                << " mask_ms=" << (t_mask_end - t_mask_begin)
                << " invalid_points=" << _last_projection_invalid_points
                << " oob_writes=" << _last_projection_oob_writes
                << " invalid_pixels=" << _invalid_idx_buffer.size() << std::endl;
    }
    _recordBufferStats(run_id);
}

void Segmentation::_recordStageTiming(double projection_ms, double tensor_ms, double run_ms, double mask_ms,
                                      size_t cloud_points) {
  auto push_with_limit = [this](std::vector<double>& hist, double v) {
    hist.push_back(v);
    if (hist.size() > _timing_window_size) {
      hist.erase(hist.begin());
    }
  };
  push_with_limit(_hist_projection_ms, projection_ms);
  push_with_limit(_hist_tensor_ms, tensor_ms);
  push_with_limit(_hist_run_ms, run_ms);
  push_with_limit(_hist_mask_ms, mask_ms);
  ++_timing_sample_count;

  const double total_ms = projection_ms + tensor_ms + run_ms + mask_ms;
  const bool slow_frame = total_ms > 120.0 || run_ms > 80.0;
  const bool periodic = (_timing_sample_count % _timing_print_interval) == 0;
  if (!slow_frame && !periodic) {
    return;
  }

  const double p50_proj = percentile(_hist_projection_ms, 0.50);
  const double p95_proj = percentile(_hist_projection_ms, 0.95);
  const double p50_tensor = percentile(_hist_tensor_ms, 0.50);
  const double p95_tensor = percentile(_hist_tensor_ms, 0.95);
  const double p50_run = percentile(_hist_run_ms, 0.50);
  const double p95_run = percentile(_hist_run_ms, 0.95);
  const double p50_mask = percentile(_hist_mask_ms, 0.50);
  const double p95_mask = percentile(_hist_mask_ms, 0.95);

  const char* reason = slow_frame ? "slow_frame" : "periodic";
  std::cout << "[Segmentation][Perf][StageTiming] "
            << "reason=" << reason
            << " sample=" << _timing_sample_count
            << " window=" << _hist_run_ms.size()
            << " pts=" << cloud_points
            << " frame_ms(total=" << total_ms
            << ",proj=" << projection_ms
            << ",tensor=" << tensor_ms
            << ",run=" << run_ms
            << ",mask=" << mask_ms << ")"
            << " p50_ms(proj=" << p50_proj
            << ",tensor=" << p50_tensor
            << ",run=" << p50_run
            << ",mask=" << p50_mask << ")"
            << " p95_ms(proj=" << p95_proj
            << ",tensor=" << p95_tensor
            << ",run=" << p95_run
            << ",mask=" << p95_mask << ")"
            << std::endl;
}

void Segmentation::_recordBufferStats(uint64_t run_id) const {
  if (_buffer_stats_interval == 0 || (run_id % _buffer_stats_interval) != 0) {
    return;
  }

  const double tensor_usage_pct =
      _input_tensor_buffer.capacity() > 0
          ? (100.0 * static_cast<double>(_input_tensor_buffer.size()) /
             static_cast<double>(_input_tensor_buffer.capacity()))
          : 0.0;
  const double invalid_usage_pct =
      _invalid_idx_buffer.capacity() > 0
          ? (100.0 * static_cast<double>(_invalid_idx_buffer.size()) /
             static_cast<double>(_invalid_idx_buffer.capacity()))
          : 0.0;
  const size_t total_pixels = static_cast<size_t>(_img_w) * static_cast<size_t>(_img_h);
  const double invalid_pixel_ratio_pct =
      total_pixels > 0
          ? (100.0 * static_cast<double>(_invalid_idx_buffer.size()) / static_cast<double>(total_pixels))
          : 0.0;

  std::cout << "[Segmentation][Perf][BufferStats] "
            << "run_id=" << run_id
            << " interval=" << _buffer_stats_interval
            << " input_tensor(size=" << _input_tensor_buffer.size()
            << ",capacity=" << _input_tensor_buffer.capacity()
            << ",usage_pct=" << tensor_usage_pct << ")"
            << " invalid_idx(size=" << _invalid_idx_buffer.size()
            << ",capacity=" << _invalid_idx_buffer.capacity()
            << ",usage_pct=" << invalid_usage_pct << ")"
            << " invalid_pixel_ratio_pct=" << invalid_pixel_ratio_pct
            << std::endl;
}

void Segmentation::speedTest(const Cloud::Ptr cloud, size_t numTests){
    std::vector<float> cloudVector;
    for (const auto& point : cloud->points) {
        cloudVector.push_back(point.x); cloudVector.push_back(point.y);
        cloudVector.push_back(point.z); cloudVector.push_back(point.intensity);
    }

    std::cout << "PROJECTION" << std::endl;
    auto netInput = _doProjection(cloudVector, cloud->width*cloud->height);
    
    std::cout << "TO TENSOR" << std::endl;
    std::vector<float> outputTensorValues(_outputTensorSize);
    std::vector<float> inputTensorValues(_inputTensorSize);
    std::vector<size_t> invalid_idxs;
    _makeTensor(netInput, inputTensorValues, invalid_idxs);

    std::cout << "SETUP" << std::endl;
    std::vector<Ort::Value> inputTensors;
    std::vector<Ort::Value> outputTensors;
    inputTensors.push_back(Ort::Value::CreateTensor<float>(
        *_memoryInfo, inputTensorValues.data(), _inputTensorSize, _inputDims.data(),
        _inputDims.size()));

    outputTensors.push_back(Ort::Value::CreateTensor<float>(
        *_memoryInfo, outputTensorValues.data(), _outputTensorSize,
        _outputDims.data(), _outputDims.size()));

    // Measure latency
    std::chrono::steady_clock::time_point begin =
        std::chrono::steady_clock::now();
    for (int i = 0; i < numTests; i++)
    {
    _session->Run(Ort::RunOptions{nullptr}, _inputNames.data(),
                inputTensors.data(), 1, _outputNames.data(),
                outputTensors.data(), 1);
    }
    std::chrono::steady_clock::time_point end =
        std::chrono::steady_clock::now();
    std::cout << "Minimum Inference Latency: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end -
                                                                       begin)
                         .count() /
                     static_cast<float>(numTests)
              << " ms" << std::endl;
}
} //namespace segmentation
