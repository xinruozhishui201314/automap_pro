#!/bin/bash
# AutoMap-Pro 快速启动脚本（增强日志版）
# 一键运行建图并查看日志

# 颜色输出
GREEN='\033[0;32m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# 显示菜单
show_menu() {
    echo -e "\n${CYAN}══════════════════════════════════════════════════${NC}"
    echo -e "${CYAN}  AutoMap-Pro 建图快速启动${NC}"
    echo -e "${CYAN}══════════════════════════════════════════════════${NC}"
    echo ""
    echo "  1) Docker 模式建图（推荐）"
    echo "  2) 本地模式建图"
    echo "  3) 查看日志摘要"
    echo "  4) 查看错误信息"
    echo "  5) 查看节点监控"
    echo "  6) 查看话题监控"
    echo "  7) 查看进度追踪"
    echo "  8) 运行诊断"
    echo "  9) 实时跟踪日志"
    echo "  0) 退出"
    echo ""
    read -p "请选择 (0-9): " choice
    echo ""
}

# 主循环
while true; do
    show_menu

    case $choice in
        1)
            echo -e "${GREEN}启动 Docker 模式建图...${NC}"
            ./run_full_mapping_docker.sh -b @data/automap_input/nya_02.bag
            ;;
        2)
            echo -e "${GREEN}启动本地模式建图...${NC}"
            ./run_full_mapping_enhanced.sh -b data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag
            ;;
        3)
            ./view_logs.sh -s
            ;;
        4)
            ./view_logs.sh -e
            ;;
        5)
            ./view_logs.sh -n
            ;;
        6)
            ./view_logs.sh -t
            ;;
        7)
            ./view_logs.sh -p
            ;;
        8)
            ./view_logs.sh -d
            ;;
        9)
            ./view_logs.sh -f
            ;;
        0)
            echo -e "${GREEN}退出${NC}"
            exit 0
            ;;
        *)
            echo -e "${BLUE}无效选择，请重试${NC}"
            ;;
    esac

    echo ""
    read -p "按 Enter 键继续..."
done
