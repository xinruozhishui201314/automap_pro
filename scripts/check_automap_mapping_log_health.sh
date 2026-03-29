#!/usr/bin/env bash
# 跑后 / CI：根据 full.log 判断「地图是否具备抑制重访重影的最小约束条件」。
# 退出码：0=通过；1=WARN（无 HBA 激光回环）；2=FAIL（HBA GPS 杆臂为零）；64=用法错误
#
# 用法:
#   scripts/check_automap_mapping_log_health.sh logs/run_YYYYMMDD_HHMMSS/full.log
# 环境变量:
#   STRICT_LOOP=1  若未出现任何 loop_added>0 的 CONSTRAINT_KPI[HBA] 行则视为失败（exit 3）

set -euo pipefail

log="${1:-}"
if [[ -z "${log}" || ! -f "${log}" ]]; then
  echo "Usage: $0 path/to/full.log" >&2
  exit 64
fi

rc=0

if grep -Fq '[HBA] GPS lever arm is zero' "${log}"; then
  echo "FAIL: [HBA] GPS lever arm is zero — 检查 gps.lever_arm_imu、离线补丁 system_config_offline_patched.yaml 与 [GPS_LEVER_ARM_CACHE]" >&2
  rc=2
fi

# 至少一行 HBA KPI 且 loop_added 含非零数字（子图级回环曾进入 HBA）
# 使用 -F：避免 [CONSTRAINT_KPI][HBA] 中方括号被 grep 当作字符类
if grep -F '[CONSTRAINT_KPI][HBA]' "${log}" | grep -qE 'loop_added=[1-9][0-9]*'; then
  echo "OK: HBA CONSTRAINT_KPI 中存在 loop_added>0（子图级激光回环曾加入）"
else
  if grep -Fq '[CONSTRAINT_KPI][HBA]' "${log}"; then
    echo "WARN: 全文无 loop_added>0 — 长轨迹重访在数学上难以保证全局激光一致；请调 loop_closure / 场景数据" >&2
    if [[ "${STRICT_LOOP:-0}" == "1" ]]; then
      echo "STRICT_LOOP=1: treating missing laser loops as failure" >&2
      rc=3
    elif [[ "${rc}" -eq 0 ]]; then
      rc=1
    fi
  else
    echo "WARN: 日志中无 CONSTRAINT_KPI[HBA]（可能日志截断或非完整建图）" >&2
    if [[ "${rc}" -eq 0 ]]; then
      rc=1
    fi
  fi
fi

# 可选：LOOP_RESOLVE 全零提示
if grep -Fq '[HBA][LOOP_RESOLVE] total=0' "${log}" && ! grep -F '[CONSTRAINT_KPI][HBA]' "${log}" | grep -qE 'loop_added=[1-9]'; then
  echo "NOTE: [HBA][LOOP_RESOLVE] total=0 与无 loop_added>0 一致时，回环解析链路未产出子图约束" >&2
fi

# V3：GPS 对齐与当前云 world→map 漂移（跳变/重影排障）
# 新日志：[V3][GPS_ALIGN_APPLIED]；旧日志可能仅有 [V3][POSE_DIAG] GPS Alignment updated
n_gps_applied=$(grep -cF '[V3][GPS_ALIGN_APPLIED]' "${log}" 2>/dev/null || echo 0)
n_gps_legacy=$(grep -cF '[V3][POSE_DIAG] GPS Alignment updated' "${log}" 2>/dev/null || echo 0)
n_pose_drift=$(grep -cE '\[V3\]\[POSE_DRIFT\]' "${log}" 2>/dev/null || echo 0)
n_odom_opt=$(grep -cF 'odom_vs_opt_trans=' "${log}" 2>/dev/null || echo 0)
if [[ "${n_pose_drift}" =~ ^[0-9]+$ ]] && [[ "${n_pose_drift}" -gt 0 ]]; then
  echo "NOTE: [V3][POSE_DRIFT]* 出现 ${n_pose_drift} 次 — 当前云在 GPS 激活下「锚点链式 world→map」与「仅用 T_map_odom」平移差 >0.5m，RViz 会 blend（见 visualization_module.h）"
  echo "      验证：grep -nE '\\[V3\\]\\[GPS_ALIGN_APPLIED\\]|\\[V3\\]\\[POSE_DRIFT\\]|odom_vs_opt_trans=' '${log}' | head -40"
fi
if [[ "${n_gps_applied}" =~ ^[0-9]+$ ]] && [[ "${n_gps_applied}" -gt 0 ]]; then
  echo "OK: [V3][GPS_ALIGN_APPLIED] ${n_gps_applied} 条（可和对齐后 POSE_DRIFT / odom_vs_opt 对照时间线）"
elif [[ "${n_gps_legacy}" =~ ^[0-9]+$ ]] && [[ "${n_gps_legacy}" -gt 0 ]]; then
  echo "NOTE: 无 GPS_ALIGN_APPLIED 标签，仅有旧式 'GPS Alignment updated' ${n_gps_legacy} 条 — 请用含新构建的日志或手工对照时间戳"
fi
if [[ "${n_odom_opt}" =~ ^[0-9]+$ ]] && [[ "${n_odom_opt}" -gt 0 ]]; then
  echo "NOTE: GHOSTING_TRACE odom_vs_opt 行共 ${n_odom_opt} — merge 时 T_map_b_optimized 与 T_odom_b 分歧（GPS/优化后常见）"
fi

exit "${rc}"
