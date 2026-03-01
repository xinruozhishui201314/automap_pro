# 使用现有 Docker 镜像指南

## 概述

已配置使用现有的 automap-env:humble Docker 镜像（22.3GB），无需重新构建。

## 镜像信息

| 属性 | 值 |
|------|-----|
| 镜像名称 | automap-env:humble |
| 版本 | latest |
| 大小 | 22.3GB |
| 内容 | ROS2 Humble + 所有依赖 |

## 快速开始

### 方法1: 使用 Makefile（推荐）⭐⭐⭐

cd /home/wqs/Documents/github/automap_pro
make docker-mapping

### 方法2: 使用脚本

cd /home/wqs/Documents/github/automap_pro
./run_full_mapping_docker.sh

## 验证镜像

查看现有镜像：
docker images | grep automap

预期输出：
REPOSITORY    TAG     IMAGE ID      CREATED       SIZE
automap-env   humble  9b192a834b7   2024-02-28   22.3GB

## 使用示例

示例1: 默认配置运行
make docker-mapping

示例2: 指定 bag 文件
./run_full_mapping_docker.sh -b @data/automap_input/nya_02.bag

示例3: 进入容器调试
./run_full_mapping_docker.sh --shell

示例4: 后台运行
./run_full_mapping_docker.sh --detach

## 快速命令速查

make docker-mapping              # Docker 一键建图
./run_full_mapping_docker.sh       # Docker 建图脚本
./run_full_mapping_docker.sh --shell  # 进入容器 shell
docker logs -f automap_mapping   # 查看容器日志
docker stop automap_mapping      # 停止容器

## 注意事项

- 脚本默认使用 automap-env:humble 镜像
- 如果镜像不存在，会提示您
- 使用 @ 前缀指定相对路径（推荐）
- 建图结果保存在 /data/automap_output/nya_02
