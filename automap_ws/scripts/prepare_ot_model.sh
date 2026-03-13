#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════════════════════
# OverlapTransformer 模型准备脚本
# 将 PyTorch 权重转换为 TorchScript (.pt) 供 LibTorch C++ 推理使用
# ══════════════════════════════════════════════════════════════════════════════
set -e

# ── 路径配置 ─────────────────────────────────────────────────────────────────
OT_DIR="/home/wqs/Documents/github/automap_pro/automap_pro/src/modular/OverlapTransformer-master"
MODEL_DIR="/data/models"
MODEL_NAME="overlapTransformer.pt"

# 检查源码目录
if [ ! -d "${OT_DIR}" ]; then
    echo "[ERROR] OverlapTransformer not found at ${OT_DIR}"
    echo "  Please clone: git clone https://github.com/haomo-ai/OverlapTransformer.git ${OT_DIR}"
    exit 1
fi

# 检查预训练权重
WEIGHTS="${OT_DIR}/model/pretrained_overlap_transformer.pth.tar"
if [ ! -f "${WEIGHTS}" ]; then
    echo "[ERROR] Pretrained weights not found: ${WEIGHTS}"
    echo "  Please download from: https://github.com/haomo-ai/OverlapTransformer/releases"
    exit 1
fi

echo "[INFO] Found weights: ${WEIGHTS}"
mkdir -p "${MODEL_DIR}"

# ── 修改 config.yml 指向正确权重 ──────────────────────────────────────────────
CONFIG="${OT_DIR}/config/config.yml"
if [ -f "${CONFIG}" ]; then
    # 更新 test_weights 路径
    sed -i "s|test_weights:.*|test_weights: \"${WEIGHTS}\"|g" "${CONFIG}"
    echo "[INFO] Updated config: ${CONFIG}"
fi

# ── 运行模型转换 ──────────────────────────────────────────────────────────────
echo "[INFO] Converting PyTorch model to TorchScript..."
cd "${OT_DIR}/OT_libtorch"

# 检查 Python 依赖
python3 -c "import torch, yaml" 2>/dev/null || {
    echo "[WARN] Installing Python dependencies..."
    pip install torch pyyaml
}

# 执行转换
python3 gen_libtorch_model.py

# 移动到目标目录
if [ -f "./overlapTransformer.pt" ]; then
    mv ./overlapTransformer.pt "${MODEL_DIR}/${MODEL_NAME}"
    echo "[SUCCESS] Model saved to: ${MODEL_DIR}/${MODEL_NAME}"
    echo ""
    echo "[Next Step] Update system_config.yaml:"
    echo "  loop_closure:"
    echo "    overlap_transformer:"
    echo "      model_path: \"${MODEL_DIR}/${MODEL_NAME}\""
else
    echo "[ERROR] Model conversion failed - overlapTransformer.pt not generated"
    exit 1
fi

# ── 验证模型 ──────────────────────────────────────────────────────────────────
echo "[INFO] Verifying model..."
python3 -c "
import torch
model = torch.jit.load('${MODEL_DIR}/${MODEL_NAME}')
model.eval()
x = torch.rand(1, 1, 64, 900)
if torch.cuda.is_available():
    model = model.cuda(); x = x.cuda()
    print('[OK] CUDA inference: output shape =', model(x).shape)
else:
    print('[OK] CPU inference: output shape =', model(x).shape)
print('[OK] Model verification passed')
"
