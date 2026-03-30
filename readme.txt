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

近期代码更新摘要（2026-03-30）
------------------------------
1) GPS / iSAM2（RC-1）
   - gps.disable_altitude_constraint（默认 true）与 gps.altitude_variance_override：
     弱 GPS 高度下放宽 Z 约束，减轻地面双层/重影。
   - incremental_optimizer / isam2_gps_manager / hba_optimizer：add 与 flush 路径
     对 Z 轴方差一致。
   - system_config_M2DGR.yaml：quality_threshold_hdop / keyframe_max_hdop 当前为 12.0
    （兼容 M2DGR HDOP~10）；Z 向重影主要靠高度方差策略而非一味收紧 HDOP。

2) V3 建图与后端
   - deferredSetupModules 注册顺序含 MapOrchestrator（在 Optimizer 与 Mapping 之间）。
   - mapping_module、optimizer_module、submap_manager：子图与优化任务调度。

3) 回环与可视化
   - loop_detector、teaser_matcher、icp_refiner：几何验证链路。
   - rviz_publisher：RViz 发布逻辑更新。

版本: 文档 bundle v2.4 | 与 Git 提交同步时请以提交说明为准。
