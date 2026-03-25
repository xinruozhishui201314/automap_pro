// Minimal LibTorch smoke: load a TorchScript .pt and optionally run one forward.
// Build: see build_cpp_jit_smoke.sh (requires LibTorch C++ matching your PyTorch version).
#include <torch/script.h>
#include <torch/torch.h>

#include <cstdlib>
#include <iostream>
#include <string>

static void usage(const char* argv0) {
  std::cerr << "usage: " << argv0 << " <model.pt> [--forward]\n"
            << "  default: only torch::jit::load + eval()\n"
            << "  --forward: run one dummy forward (needs I/O shapes matching the traced graph)\n";
}

int main(int argc, char** argv) {
  if (argc < 2) {
    usage(argv[0]);
    return 1;
  }
  const std::string path = argv[1];
  bool do_forward = false;
  for (int i = 2; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--forward") {
      do_forward = true;
    } else {
      usage(argv[0]);
      return 1;
    }
  }

  try {
    torch::jit::Module mod = torch::jit::load(path);
    mod.eval();
    std::cout << "[cpp] torch::jit::load OK: " << path << "\n";
    if (!do_forward) {
      return 0;
    }
    // TraceWrapper 输入: points [N,C], normal [N,3], batch_idx [N], labels [N]
    const int64_t n = 4096;
    const int64_t c = 13;
    auto opts_f = torch::TensorOptions().dtype(torch::kFloat32);
    auto opts_l = torch::TensorOptions().dtype(torch::kInt64);
    torch::Tensor points = torch::randn({n, c}, opts_f);
    torch::Tensor normal = torch::randn({n, 3}, opts_f);
    torch::Tensor batch_idx = torch::zeros({n}, opts_l);
    torch::Tensor labels = torch::zeros({n}, opts_l);
    std::vector<torch::jit::IValue> inputs = {points, normal, batch_idx, labels};
    torch::NoGradGuard guard;
    auto out = mod.forward(inputs);
    if (out.isTensor()) {
      auto t = out.toTensor();
      std::cout << "[cpp] forward OK, logits shape: " << t.sizes() << "\n";
    } else {
      std::cout << "[cpp] forward returned non-Tensor (tuple/dict); load path OK.\n";
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "[cpp] FAILED: " << e.what() << "\n";
    return 2;
  }
}
