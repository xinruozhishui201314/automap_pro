#!/usr/bin/env bash
# 自动验证：建图 launch 是否使用 system_config.yaml 注入参数（fast_livo / overlap_transformer）
# 用法：从仓库根目录或 automap_pro 目录执行
#   ./automap_pro/scripts/verify_system_config_launch.sh
#   ./automap_pro/scripts/verify_system_config_launch.sh [config.yaml 路径]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 仓库根或 automap_pro 包根
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." 2>/dev/null && pwd)"
AUTOMAP_PRO_DIR="${REPO_ROOT:-$SCRIPT_DIR/..}"
LAUNCH_DIR="$AUTOMAP_PRO_DIR/automap_pro/launch"
CONFIG_ARG="${1:-$AUTOMAP_PRO_DIR/automap_pro/config/system_config.yaml}"
# 转为绝对路径，以便在 cd 到 LAUNCH_DIR 后 Python 仍能找到
if [[ "$CONFIG_ARG" != /* ]]; then
    CONFIG_ABS="$AUTOMAP_PRO_DIR/$CONFIG_ARG"
else
    CONFIG_ABS="$CONFIG_ARG"
fi
CONFIG_ARG="$CONFIG_ABS"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NC='\033[0m'

pass() { echo -e "${GREEN}[PASS]${NC} $*"; }
fail() { echo -e "${RED}[FAIL]${NC} $*"; exit 1; }
warn() { echo -e "${YELLOW}[WARN]${NC} $*"; }

echo "=========================================="
echo "  system_config 建图 launch 自动验证"
echo "=========================================="
echo "LAUNCH_DIR=$LAUNCH_DIR"
echo "CONFIG=$CONFIG_ARG"
echo ""

# 1) 源码 launch 必须包含 params_from_system_config / OpaqueFunction
for f in automap_offline.launch.py automap_online.launch.py; do
    path="$LAUNCH_DIR/$f"
    if [ ! -f "$path" ]; then
        warn "Launch 文件不存在: $path"
        continue
    fi
    if grep -q "params_from_system_config\|OpaqueFunction" "$path" 2>/dev/null; then
        pass "源码 launch 使用 system_config: $f"
    else
        fail "源码 launch 未使用 system_config（缺少 params_from_system_config/OpaqueFunction）: $path"
    fi
done

# 2) params_from_system_config.py 存在且可导入
PARAMS_PY="$LAUNCH_DIR/params_from_system_config.py"
if [ ! -f "$PARAMS_PY" ]; then
    fail "params_from_system_config.py 不存在: $PARAMS_PY"
fi
pass "params_from_system_config.py 存在"

# 3) 用 Python 加载 config 并检查 get_fast_livo2_params 产出（必填字符串非空）
if [ ! -f "$CONFIG_ARG" ]; then
    warn "配置文件不存在，跳过参数产出检查: $CONFIG_ARG"
else
    cd "$LAUNCH_DIR"
    out=$(python3 - "$CONFIG_ARG" << 'PYEOF'
import sys
config_path = sys.argv[1] if len(sys.argv) > 1 else ""
sys.path.insert(0, '.')
try:
    from params_from_system_config import load_system_config, get_fast_livo2_params
    config = load_system_config(config_path)
    params = get_fast_livo2_params(config)
    common = params.get('common', {})
    lid = common.get('lid_topic', '')
    imu = common.get('imu_topic', '')
    img = common.get('img_topic', '')
    evo = params.get('evo', {})
    seq = evo.get('seq_name', '') if isinstance(evo, dict) else ''
    err = []
    if not (lid and isinstance(lid, str)): err.append('common.lid_topic')
    if not (imu and isinstance(imu, str)): err.append('common.imu_topic')
    if not (img and isinstance(img, str)): err.append('common.img_topic')
    if not (seq and isinstance(seq, str)): err.append('evo.seq_name')
    if err:
        print('MISSING_STRINGS:' + ','.join(err))
        sys.exit(1)
    print('OK')
except Exception as e:
    print('ERROR:' + str(e))
    sys.exit(2)
PYEOF
)
    ex=$?
    if [ "$ex" -eq 0 ] && echo "$out" | grep -q '^OK$'; then
        pass "get_fast_livo2_params 产出必填字符串非空 (config=$CONFIG_ARG)"
    elif [ "$ex" -eq 1 ]; then
        fail "get_fast_livo2_params 缺少必填字符串: $out"
    else
        fail "params_from_system_config 执行异常: $out"
    fi
fi

# 4) 若 install 存在，检查其 launch 是否也已更新（可选，仅提示）
INSTALL_LAUNCH=""
if [ -n "$WORKSPACE" ] && [ -f "$WORKSPACE/install/automap_pro/share/automap_pro/launch/automap_offline.launch.py" ]; then
    INSTALL_LAUNCH="$WORKSPACE/install/automap_pro/share/automap_pro/launch/automap_offline.launch.py"
elif [ -f "$AUTOMAP_PRO_DIR/automap_ws/install/automap_pro/share/automap_pro/launch/automap_offline.launch.py" ]; then
    INSTALL_LAUNCH="$AUTOMAP_PRO_DIR/automap_ws/install/automap_pro/share/automap_pro/launch/automap_offline.launch.py"
fi
if [ -n "$INSTALL_LAUNCH" ]; then
    if grep -q "params_from_system_config\|OpaqueFunction" "$INSTALL_LAUNCH" 2>/dev/null; then
        pass "install 空间 launch 已含 system_config 逻辑（与源码一致）"
    else
        warn "install 空间 launch 仍为旧版（未含 system_config）；建图脚本将优先使用源码 launch，无需强制 rebuild"
    fi
fi

echo ""
echo -e "${GREEN}=========================================="
echo "  所有检查通过：建图将使用 system_config 注入参数"
echo "==========================================${NC}"
