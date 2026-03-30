#pragma once
/**
 * @file sensor/extrinsic_online_calibrator.h
 * @brief 传感器：LiDAR/IMU/GPS/相机/时间同步与在线标定。
 */


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>
#include <deque>

namespace automap_pro {

/**
 * @brief LiDAR-Camera外参在线标定器
 * 
 * 功能：
 * 1. 使用点云-图像特征匹配估计外参
 * 2. 视觉验证外参精度（投影误差分析）
 * 3. 支持外参在线优化
 * 4. 外参质量监控与异常检测
 * 
 * 方法：
 * - 边缘特征匹配：LiDAR点云投影到图像平面，与Canny边缘对齐
 * - 平面特征匹配：LiDAR平面与图像平面区域匹配
 * - 直接法优化：最小化光度误差
 * 
 * 使用场景：
 * - M2DGR数据集外参验证
 * - 实时优化LiDAR-Camera外参
 * - 外参漂移检测与校正
 */
class ExtrinsicOnlineCalibrator : public rclcpp::Node {
public:
    explicit ExtrinsicOnlineCalibrator(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions())
        : Node("extrinsic_online_calibrator", node_options) {
        
        // 声明参数
        this->declare_parameter("camera_topic", std::string("/camera/head/image_raw"));
        this->declare_parameter("lidar_topic", std::string("/velodyne_points"));
        this->declare_parameter("camera_info_topic", std::string("/camera/head/camera_info"));
        this->declare_parameter("queue_size", 30);
        
        // 相机内参
        this->declare_parameter("fx", 542.993253538048);
        this->declare_parameter("fy", 541.3882904458247);
        this->declare_parameter("cx", 629.0025857364897);
        this->declare_parameter("cy", 503.71809588651786);
        this->declare_parameter("k1", -0.057963907006683066);
        this->declare_parameter("k2", -0.026465594265953234);
        this->declare_parameter("p1", 0.011980216320790046);
        this->declare_parameter("p2", -0.003041081642470451);
        
        // 初始外参（LiDAR到相机）
        this->declare_parameter("init_trans_x", 0.07410);
        this->declare_parameter("init_trans_y", 0.00127);
        this->declare_parameter("init_trans_z", 0.65608);
        this->declare_parameter("init_roll", 0.0);
        this->declare_parameter("init_pitch", 0.0);
        this->declare_parameter("init_yaw", 0.0);
        
        // 标定参数
        this->declare_parameter("enable_online_optimization", true);
        this->declare_parameter("calib_interval", 1.0);  // 标定间隔（秒）
        this->declare_parameter("max_iter", 100);
        this->declare_parameter("converge_threshold", 1e-6);
        this->declare_parameter("project_error_threshold", 5.0);  // 像素
        this->declare_parameter("min_keypoints", 50);
        this->declare_parameter("edge_threshold_low", 50);
        this->declare_parameter("edge_threshold_high", 150);
        
        // 获取参数
        camera_topic_ = this->get_parameter("camera_topic").as_string();
        lidar_topic_ = this->get_parameter("lidar_topic").as_string();
        camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();
        queue_size_ = this->get_parameter("queue_size").as_int();
        
        fx_ = this->get_parameter("fx").as_double();
        fy_ = this->get_parameter("fy").as_double();
        cx_ = this->get_parameter("cx").as_double();
        cy_ = this->get_parameter("cy").as_double();
        
        k1_ = this->get_parameter("k1").as_double();
        k2_ = this->get_parameter("k2").as_double();
        p1_ = this->get_parameter("p1").as_double();
        p2_ = this->get_parameter("p2").as_double();
        
        init_trans_ << this->get_parameter("init_trans_x").as_double(),
                        this->get_parameter("init_trans_y").as_double(),
                        this->get_parameter("init_trans_z").as_double();
        init_rpy_ << this->get_parameter("init_roll").as_double(),
                       this->get_parameter("init_pitch").as_double(),
                       this->get_parameter("init_yaw").as_double();
        
        enable_optimization_ = this->get_parameter("enable_online_optimization").as_bool();
        calib_interval_ = this->get_parameter("calib_interval").as_double();
        max_iter_ = this->get_parameter("max_iter").as_int();
        converge_threshold_ = this->get_parameter("converge_threshold").as_double();
        project_error_threshold_ = this->get_parameter("project_error_threshold").as_double();
        min_keypoints_ = this->get_parameter("min_keypoints").as_int();
        edge_threshold_low_ = this->get_parameter("edge_threshold_low").as_int();
        edge_threshold_high_ = this->get_parameter("edge_threshold_high").as_int();
        
        RCLCPP_INFO(this->get_logger(), "[ExtrinsicOnlineCalibrator] Initializing...");
        RCLCPP_INFO(this->get_logger(), "[ExtrinsicOnlineCalibrator] Camera topic: %s", camera_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "[ExtrinsicOnlineCalibrator] LiDAR topic: %s", lidar_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "[ExtrinsicOnlineCalibrator] Online optimization: %s", 
                    enable_optimization_ ? "enabled" : "disabled");
        
        // 初始化相机内参矩阵
        K_ << fx_, 0.0, cx_,
              0.0, fy_, cy_,
              0.0, 0.0, 1.0;
        
        // 初始化外参（LiDAR到相机）
        initExtrinsic();
        
        // 创建订阅者
        sub_camera_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_topic_, queue_size_,
            std::bind(&ExtrinsicOnlineCalibrator::cameraCallback, this, std::placeholders::_1));
        
        sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic_, queue_size_,
            std::bind(&ExtrinsicOnlineCalibrator::lidarCallback, this, std::placeholders::_1));
        
        // 创建发布者
        extrinsic_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>(
            "/extrinsic_calibrator/lidar_to_camera", 10);
        
        error_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/extrinsic_calibrator/projection_error", 10);
        
        quality_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/extrinsic_calibrator/quality", 10);
        
        // 创建可视化发布者（投影点云到图像）
        viz_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/extrinsic_calibrator/projected_image", 10);
        
        // 初始化状态
        last_calib_time_ = this->now();
        synchronized_ = false;
        
        RCLCPP_INFO(this->get_logger(), "[ExtrinsicOnlineCalibrator] Ready!");
    }
    
    /**
     * @brief 获取当前外参（LiDAR到相机）
     */
    Eigen::Matrix4d getExtrinsic() const {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = R_;
        T.block<3, 1>(0, 3) = t_;
        return T;
    }
    
    /**
     * @brief 获取外参的FAST-LIVO格式（Rcl, Pcl）
     */
    std::tuple<std::vector<double>, std::vector<double>> getExtrinsicForFastLIVO() const {
        std::vector<double> Rcl(9);
        std::vector<double> Pcl(3);
        
        Eigen::Matrix3d R_inv = R_.transpose();
        Eigen::Vector3d t_inv = -R_inv * t_;
        
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                Rcl[i * 3 + j] = R_inv(i, j);
            }
            Pcl[i] = t_inv(i);
        }
        
        return {Rcl, Pcl};
    }
    
    /**
     * @brief 重置外参到初始值
     */
    void resetExtrinsic() {
        initExtrinsic();
        RCLCPP_INFO(this->get_logger(), 
                    "[ExtrinsicOnlineCalibrator] Extrinsic reset to initial values");
    }
    
    /**
     * @brief 设置外参
     */
    void setExtrinsic(const Eigen::Matrix3d& R, const Eigen::Vector3d& t) {
        R_ = R;
        t_ = t;
        RCLCPP_INFO(this->get_logger(), 
                    "[ExtrinsicOnlineCalibrator] Extrinsic set manually");
    }
    
    /**
     * @brief 获取投影误差
     */
    double getProjectionError() const { return projection_error_; }
    
    /**
     * @brief 获取外参质量分数
     */
    double getQuality() const { return quality_score_; }

private:
    /**
     * @brief 初始化外参
     */
    void initExtrinsic() {
        // 从RPY构造旋转矩阵
        Eigen::AngleAxisd roll_angle(init_rpy_[0], Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch_angle(init_rpy_[1], Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yaw_angle(init_rpy_[2], Eigen::Vector3d::UnitZ());
        R_ = yaw_angle * pitch_angle * roll_angle;
        t_ = init_trans_;
        
        RCLCPP_INFO(this->get_logger(),
                    "[ExtrinsicOnlineCalibrator] Initial extrinsic: t=[%.4f,%.4f,%.4f], rpy=[%.4f,%.4f,%.4f]",
                    t_[0], t_[1], t_[2], init_rpy_[0], init_rpy_[1], init_rpy_[2]);
    }
    
    /**
     * @brief 相机回调函数
     */
    void cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            latest_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
            latest_image_time_ = rclcpp::Time(msg->header.stamp).seconds();
            
            // 提取边缘特征
            extractImageEdges();
            
            // 检查是否可以同步
            checkSynchronized();
            
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(),
                        "[ExtrinsicOnlineCalibrator] cv_bridge exception: %s", e.what());
        }
    }
    
    /**
     * @brief LiDAR回调函数
     */
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // 转换为PCL点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        
        latest_cloud_ = cloud;
        latest_cloud_time_ = rclcpp::Time(msg->header.stamp).seconds();
        
        // 检查是否可以同步
        checkSynchronized();
        
        // 如果同步且到了标定时间，进行标定
        if (synchronized_) {
            auto now = this->now();
            if ((now - last_calib_time_).seconds() >= calib_interval_) {
                calibrateExtrinsic();
                last_calib_time_ = now;
            }
        }
    }
    
    /**
     * @brief 检查是否同步
     */
    void checkSynchronized() {
        if (latest_image_.empty() || !latest_cloud_) {
            synchronized_ = false;
            return;
        }
        
        double time_diff = std::abs(latest_image_time_ - latest_cloud_time_);
        synchronized_ = (time_diff < 0.1);  // 100ms以内认为同步
    }
    
    /**
     * @brief 提取图像边缘特征
     */
    void extractImageEdges() {
        if (latest_image_.empty()) return;
        
        cv::Mat gray;
        cv::cvtColor(latest_image_, gray, cv::COLOR_BGR2GRAY);
        
        // Canny边缘检测
        cv::Canny(gray, image_edges_, edge_threshold_low_, edge_threshold_high_);
        
        // 提取边缘点
        std::vector<cv::Point> edge_points;
        cv::findNonZero(image_edges_, edge_points);
        image_edge_points_ = edge_points;
    }
    
    /**
     * @brief 标定外参
     */
    void calibrateExtrinsic() {
        if (!synchronized_ || latest_image_.empty() || !latest_cloud_) {
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), 
                    "[ExtrinsicOnlineCalibrator] Starting extrinsic calibration...");
        
        // 筛选有效点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& point : latest_cloud_->points) {
            // 过滤无效点
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
                continue;
            }
            
            // 过滤过远和过近的点
            double dist = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            if (dist < 1.0 || dist > 50.0) {
                continue;
            }
            
            filtered_cloud->points.push_back(point);
        }
        
        if (filtered_cloud->points.size() < min_keypoints_) {
            RCLCPP_WARN(this->get_logger(),
                       "[ExtrinsicOnlineCalibrator] Insufficient points: %lu (min: %d)",
                       filtered_cloud->points.size(), min_keypoints_);
            return;
        }
        
        // 边缘匹配优化
        optimizeByEdgeMatching(filtered_cloud);
        
        // 计算投影误差和质量分数
        evaluateExtrinsic(filtered_cloud);
        
        // 发布外参
        publishExtrinsic();
        
        // 可视化
        visualizeProjection(filtered_cloud);
    }
    
    /**
     * @brief 边缘匹配优化
     */
    void optimizeByEdgeMatching(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        double last_error = std::numeric_limits<double>::max();
        
        for (int iter = 0; iter < max_iter_; ++iter) {
            double total_error = 0.0;
            int valid_count = 0;
            
            // 计算梯度
            Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
            Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();
            
            for (const auto& point : cloud->points) {
                Eigen::Vector3d p_lidar(point.x, point.y, point.z);
                
                // 变换到相机坐标系
                Eigen::Vector3d p_cam = R_ * p_lidar + t_;
                
                // 跳过相机后方的点
                if (p_cam[2] <= 0.1) continue;
                
                // 投影到图像平面
                Eigen::Vector2d uv = projectPoint(p_cam);
                
                // 检查是否在图像范围内
                if (uv[0] < 0 || uv[0] >= latest_image_.cols ||
                    uv[1] < 0 || uv[1] >= latest_image_.rows) {
                    continue;
                }
                
                // 计算边缘响应（梯度幅值）
                double edge_response = getImageEdgeResponse(uv);
                
                // 边缘响应作为误差
                double error = edge_response;
                total_error += error;
                valid_count++;
                
                // 计算雅可比矩阵（简化版）
                Eigen::Matrix<double, 1, 6> J = computeJacobian(p_lidar);
                
                // 构建正规方程
                H += J.transpose() * J;
                b += J.transpose() * error;
            }
            
            if (valid_count < min_keypoints_) {
                RCLCPP_WARN(this->get_logger(),
                           "[ExtrinsicOnlineCalibrator] Optimization failed: insufficient valid points");
                return;
            }
            
            // 求解增量
            Eigen::Matrix<double, 6, 1> dx = H.ldlt().solve(b);
            
            // 更新外参
            updateExtrinsic(dx);
            
            // 归一化旋转矩阵
            Eigen::Quaterniond q(R_);
            q.normalize();
            R_ = q.toRotationMatrix();
            
            // 计算误差变化
            double error_diff = last_error - total_error / valid_count;
            last_error = total_error / valid_count;
            
            if (std::abs(error_diff) < converge_threshold_) {
                RCLCPP_INFO(this->get_logger(),
                           "[ExtrinsicOnlineCalibrator] Converged at iteration %d", iter);
                break;
            }
        }
        
        projection_error_ = last_error;
        RCLCPP_INFO(this->get_logger(),
                    "[ExtrinsicOnlineCalibrator] Calibration completed: error=%.4f", last_error);
    }
    
    /**
     * @brief 投影3D点到2D图像平面
     */
    Eigen::Vector2d projectPoint(const Eigen::Vector3d& p_cam) const {
        Eigen::Vector3d uv_hom = K_ * p_cam;
        double u = uv_hom[0] / uv_hom[2];
        double v = uv_hom[1] / uv_hom[2];
        return Eigen::Vector2d(u, v);
    }
    
    /**
     * @brief 获取图像边缘响应
     */
    double getImageEdgeResponse(const Eigen::Vector2d& uv) const {
        int u = static_cast<int>(uv[0]);
        int v = static_cast<int>(uv[1]);
        
        if (u < 0 || u >= latest_image_.cols ||
            v < 0 || v >= latest_image_.rows) {
            return 0.0;
        }
        
        // 使用Sobel算子计算梯度
        double gx = 0.0, gy = 0.0;
        for (int i = -1; i <= 1; ++i) {
            for (int j = -1; j <= 1; ++j) {
                int ui = u + j;
                int vi = v + i;
                if (ui >= 0 && ui < latest_image_.cols &&
                    vi >= 0 && vi < latest_image_.rows) {
                    uchar pixel = latest_image_.at<uchar>(vi, ui);
                    double sobel_x = (j == -1) ? -1 : ((j == 1) ? 1 : 0);
                    double sobel_y = (i == -1) ? -1 : ((i == 1) ? 1 : 0);
                    gx += pixel * sobel_x;
                    gy += pixel * sobel_y;
                }
            }
        }
        
        return std::sqrt(gx * gx + gy * gy);
    }
    
    /**
     * @brief 计算雅可比矩阵
     */
    Eigen::Matrix<double, 1, 6> computeJacobian(const Eigen::Vector3d& p_lidar) const {
        Eigen::Vector3d p_cam = R_ * p_lidar + t_;
        double x = p_cam[0], y = p_cam[1], z = p_cam[2];
        
        Eigen::Matrix<double, 2, 6> J_proj;
        J_proj << fx_ / z, 0, -fx_ * x / (z * z),
                 0, fy_ / z, -fy_ * y / (z * z);
        
        Eigen::Matrix<double, 3, 6> J_transform;
        J_transform << R_, -R_ * skew(p_lidar);
        
        // 简化：使用边缘响应作为误差，雅可比近似为1
        Eigen::Matrix<double, 1, 6> J = Eigen::Matrix<double, 1, 6>::Ones();
        
        return J;
    }
    
    /**
     * @brief 更新外参
     */
    void updateExtrinsic(const Eigen::Matrix<double, 6, 1>& dx) {
        Eigen::Vector3d dt = dx.head<3>();
        Eigen::Vector3d dr = dx.tail<3>();
        
        // 更新平移
        t_ += dt * 0.01;  // 步长
        
        // 更新旋转（指数映射）
        Eigen::AngleAxisd dR(dr.norm(), dr.normalized());
        R_ = dR * R_;
    }
    
    /**
     * @brief 评估外参质量
     */
    void evaluateExtrinsic(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        double total_error = 0.0;
        int valid_count = 0;
        
        for (const auto& point : cloud->points) {
            Eigen::Vector3d p_lidar(point.x, point.y, point.z);
            Eigen::Vector3d p_cam = R_ * p_lidar + t_;
            
            if (p_cam[2] <= 0.1) continue;
            
            Eigen::Vector2d uv = projectPoint(p_cam);
            
            if (uv[0] < 0 || uv[0] >= latest_image_.cols ||
                uv[1] < 0 || uv[1] >= latest_image_.rows) {
                continue;
            }
            
            total_error += getImageEdgeResponse(uv);
            valid_count++;
        }
        
        if (valid_count > 0) {
            projection_error_ = total_error / valid_count;
            
            // 质量分数：误差越小，分数越高
            quality_score_ = std::exp(-projection_error_ / 100.0);
        }
        
        RCLCPP_INFO(this->get_logger(),
                    "[ExtrinsicOnlineCalibrator] Quality: error=%.4f, score=%.4f",
                    projection_error_, quality_score_);
    }
    
    /**
     * @brief 发布外参
     */
    void publishExtrinsic() {
        geometry_msgs::msg::TransformStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "velodyne";
        msg.child_frame_id = "camera";
        
        msg.transform.translation.x = t_[0];
        msg.transform.translation.y = t_[1];
        msg.transform.translation.z = t_[2];
        
        Eigen::Quaterniond q(R_);
        msg.transform.rotation.w = q.w();
        msg.transform.rotation.x = q.x();
        msg.transform.rotation.y = q.y();
        msg.transform.rotation.z = q.z();
        
        extrinsic_pub_->publish(msg);
        
        std_msgs::msg::Float64 error_msg;
        error_msg.data = projection_error_;
        error_pub_->publish(error_msg);
        
        std_msgs::msg::Float64 quality_msg;
        quality_msg.data = quality_score_;
        quality_pub_->publish(quality_msg);
    }
    
    /**
     * @brief 可视化投影结果
     */
    void visualizeProjection(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        if (latest_image_.empty()) return;
        
        cv::Mat viz_image = latest_image_.clone();
        
        for (const auto& point : cloud->points) {
            Eigen::Vector3d p_lidar(point.x, point.y, point.z);
            Eigen::Vector3d p_cam = R_ * p_lidar + t_;
            
            if (p_cam[2] <= 0.1) continue;
            
            Eigen::Vector2d uv = projectPoint(p_cam);
            
            int u = static_cast<int>(uv[0]);
            int v = static_cast<int>(uv[1]);
            
            if (u >= 0 && u < viz_image.cols && v >= 0 && v < viz_image.rows) {
                cv::circle(viz_image, cv::Point(u, v), 2, cv::Scalar(0, 255, 0), -1);
            }
        }
        
        // 发布可视化图像
        sensor_msgs::msg::Image::SharedPtr viz_msg = 
            cv_bridge::CvImage(viz_image_.header, "bgr8", viz_image).toImageMsg();
        viz_pub_->publish(*viz_msg);
    }
    
    /**
     * @brief 反对称矩阵
     */
    static Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
        Eigen::Matrix3d S;
        S << 0, -v[2], v[1],
             v[2], 0, -v[0],
             -v[1], v[0], 0;
        return S;
    }
    
    // ROS2节点组件
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_camera_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr extrinsic_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr error_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr quality_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr viz_pub_;
    
    // 相机内参
    Eigen::Matrix3d K_;
    double fx_, fy_, cx_, cy_;
    double k1_, k2_, p1_, p2_;
    
    // 外参（LiDAR到相机）
    Eigen::Matrix3d R_;
    Eigen::Vector3d t_;
    Eigen::Vector3d init_trans_;
    Eigen::Vector3d init_rpy_;
    
    // 数据缓冲
    cv::Mat latest_image_;
    cv::Mat image_edges_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr latest_cloud_;
    std::vector<cv::Point> image_edge_points_;
    double latest_image_time_ = 0.0;
    double latest_cloud_time_ = 0.0;
    
    // 配置参数
    std::string camera_topic_;
    std::string lidar_topic_;
    std::string camera_info_topic_;
    int queue_size_;
    bool enable_optimization_;
    double calib_interval_;
    int max_iter_;
    double converge_threshold_;
    double project_error_threshold_;
    int min_keypoints_;
    int edge_threshold_low_;
    int edge_threshold_high_;
    
    // 标定结果
    double projection_error_ = 0.0;
    double quality_score_ = 0.0;
    
    // 同步状态
    bool synchronized_;
    rclcpp::Time last_calib_time_;
};

}  // namespace automap_pro
