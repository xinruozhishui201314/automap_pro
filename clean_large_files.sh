#!/bin/bash
set -e

echo "========================================="
echo "开始清理 Git 仓库中的大文件"
echo "========================================="

# 检查是否在 Git 仓库中
if ! git rev-parse --git-dir > /dev/null 2>&1; then
    echo "错误: 当前目录不是 Git 仓库"
    exit 1
fi

echo ""
echo "当前仓库大小:"
du -sh .git

echo ""
echo "清理前警告:"
echo "此操作将永久删除历史记录中的大文件"
echo "建议先备份: git clone --bare . ../backup-$(date +%Y%m%d-%H%M%S).git"
echo ""
read -p "是否继续? (yes/no): " confirm

if [ "$confirm" != "yes" ]; then
    echo "操作已取消"
    exit 0
fi

echo ""
echo "========================================="
echo "步骤 1: 创建路径清单"
echo "========================================="

# 创建要移除的文件路径清单
cat > paths-to-remove.txt << 'EOF'
# ROS bag 文件
data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag

# 模型权重文件
OverlapTransformer-master/model/pretrained_overlap_transformer.pth.tar

# Docker 依赖的二进制文件
docker/deps/ispc/bin/ispc
docker/deps/downloads/cmake-3.24.0-linux-x86_64.tar.gz
docker/deps/downloads/ispc-v1.16.1-linux.tar.gz
docker/deps/cmake/bin/cmake-gui
docker/deps/cmake/bin/ctest
docker/deps/cmake/bin/cpack
docker/deps/cmake/bin/cmake
docker/deps/cmake/bin/ccmake

# 点云数据文件
TEASER-plusplus-master/test/teaser/data/uw-rgbdv2-01.ply

# GIF 动画
OverlapTransformer-master/query_database_haomo.gif
OverlapTransformer-master/query_database_kitti.gif
TEASER-plusplus-master/examples/teaser_python_3dsmooth/3dsmooth_example.gif

# PDF 文档
fast-livo2-humble/Supplementary/LIVO2_supplementary.pdf

# 示例数据文件
docker/deps/ispc/examples/cpu/deferred/data/pp1920x1200.bin
docker/deps/ispc/examples/cpu/deferred/data/pp1280x720.bin
docker/deps/ispc/examples/cpu/volume_rendering/density_highres.vol
docker/deps/ispc/examples/cpu/rt/sponza.bvh

# 其他示例数据
OverlapTransformer-master/OT_libtorch/ws/000000.bin
OverlapTransformer-master/demo/scans/000000.bin
OverlapTransformer-master/demo/scans/000005.bin
OverlapTransformer-master/demo/scans/000015.bin

# TEASER 示例点云
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_2.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_36.ply
TEASER-plusplus-master/examples/example_data/bun_zipper_res3.ply
TEASER-plusplus-master/examples/teaser_python_fpfh_icp/data/cloud_bin_0.ply
TEASER-plusplus-master/examples/teaser_python_fpfh_icp/data/cloud_bin_4.ply
TEASER-plusplus-master/test/benchmark/data/benchmark_1/dst.ply
TEASER-plusplus-master/test/benchmark/data/benchmark_1/src.ply
TEASER-plusplus-master/test/benchmark/data/benchmark_2/dst.ply
TEASER-plusplus-master/test/benchmark/data/benchmark_2/src.ply
TEASER-plusplus-master/test/benchmark/data/benchmark_3/dst.ply
TEASER-plusplus-master/test/benchmark/data/benchmark_3/src.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_0.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_1.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_10.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_100.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_101.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_102.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_103.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_104.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_105.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_106.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_107.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_108.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_109.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_11.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_110.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_111.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_112.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_113.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_114.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_115.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_116.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_117.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_118.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_119.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_12.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_13.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_14.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_15.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_16.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_17.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_18.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_19.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_20.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_21.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_22.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_23.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_24.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_25.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_26.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_27.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_28.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_29.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_3.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_30.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_31.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_32.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_33.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_34.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_35.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_36.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_37.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_38.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_39.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_4.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_40.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_41.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_42.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_43.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_44.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_45.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_46.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_47.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_48.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_49.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_5.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_50.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_51.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_52.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_53.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_54.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_55.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_56.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_57.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_58.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_59.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_6.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_60.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_61.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_62.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_63.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_64.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_65.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_66.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_67.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_68.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_69.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_70.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_71.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_72.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_73.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_74.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_75.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_76.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_77.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_78.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_79.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_8.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_80.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_81.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_82.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_83.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_84.ply
TEASER-plusplus-master/examples/example_data/3droll_sample/cloud_bin_85.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_86.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_87.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_88.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_89.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_90.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_91.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_92.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_93.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_94.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_95.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_96.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_97.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_98.ply
TEASER-plusplus-master/examples/example_data/3dmatch_sample/cloud_bin_99.ply

# HAomo 数据集 GIF
OverlapTransformer-master/Haomo_Dataset/dataset_short_term.gif
EOF

echo "路径清单已创建: paths-to-remove.txt"
echo "总共包含 $(grep -c -v '^#' paths-to-remove.txt | grep -c -v '^$') 个路径"

echo ""
echo "========================================="
echo "步骤 2: 使用 git-filter-repo 清理历史"
echo "========================================="

# 使用 git-filter-repo 清理历史
echo "开始清理，这可能需要几分钟..."
git filter-repo --invert-paths --paths-from-file paths-to-remove.txt

echo ""
echo "========================================="
echo "清理完成!"
echo "========================================="

echo ""
echo "清理后仓库大小:"
du -sh .git

echo ""
echo "========================================="
echo "步骤 3: 强制垃圾回收"
echo "========================================="

git reflog expire --expire=now --all
git gc --prune=now --aggressive

echo ""
echo "垃圾回收完成!"
echo ""
echo "最终仓库大小:"
du -sh .git

echo ""
echo "========================================="
echo "后续步骤:"
echo "========================================="
echo "1. 检查清理结果:"
echo "   git log --oneline | head -5"
echo ""
echo "2. 强制推送到远程（谨慎操作）:"
echo "   git push --force-with-lease origin main"
echo ""
echo "注意: 这将改写远程仓库历史，请确保团队成员已知晓！"
