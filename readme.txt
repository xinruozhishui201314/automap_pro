AutoMap-Pro — 高精度自动化点云建图系统
========================================

完整说明（架构图、语义管线、故障排查）请阅读仓库根目录的 README.md。
本文档为纯文本速查与近期变更摘要。

主入口（推荐）
--------------
  仓库根目录执行:
    bash run_automap.sh
  离线回放示例:
    bash run_automap.sh --offline \
      --bag-file data/automap_input/M2DGR/street_03_ros2 \
      --config system_config_M2DGR.yaml \
      --bag-rate 0.5
  后端 GDB:
    bash run_automap.sh --offline --bag-file <path> --config <yaml> --gdb

主配置
------
  automap_pro/config/system_config.yaml
  数据集示例: automap_pro/config/system_config_M2DGR.yaml

日志
----
  logs/full.log, logs/build.log

文档索引
--------
  docs/README.md
  docs/BUILD_DEPLOY_RUN.md
  QUICK_START.md
  automap_pro/docs/TROUBLESHOOTING.md

近期代码更新摘要（2026-03-29）
------------------------------
1) GPS / iSAM2（RC-1）
   - 配置项 gps.disable_altitude_constraint（默认 true）与
     gps.altitude_variance_override：弱 GPS 高度下放宽 Z 约束，减轻地面双层/重影。
   - incremental_optimizer / isam2_gps_manager / hba_optimizer：add 与 flush 路径
     对 Z 轴方差处理一致。
   - system_config_M2DGR.yaml：收紧 quality_threshold_hdop / keyframe_max_hdop。

2) V3 建图与后端
   - mapping_module、optimizer_module、submap_manager：子图与优化任务调度相关增强。

3) 回环与可视化
   - loop_detector、teaser_matcher：回环检测与几何验证链路调整。
   - rviz_publisher：RViz 发布逻辑更新。

版本: 文档 bundle v2.3 | 与 Git 提交同步时请以提交说明为准。
