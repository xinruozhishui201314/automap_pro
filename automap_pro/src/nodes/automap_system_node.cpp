#include "automap_pro/system/automap_system.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    // 启用进程内通信（Intra-Process Communication）
    // 当与 fast_livo 在同一 Component Container 中运行时，自动零拷贝
    options.use_intra_process_comms(true);

    auto node = std::make_shared<automap_pro::AutoMapSystem>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
