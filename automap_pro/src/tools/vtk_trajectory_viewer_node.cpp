/**
 * VTK 轨迹曲线显示工具（工程内 C++ 节点）
 *
 * 订阅 /automap/optimized_path（HBA 轨迹）与 /automap/gps_keyframe_path（GPS 轨迹），
 * 使用 VTK 显示两条 3D 曲线及偏差曲线（偏差随里程）。
 *
 * 用法：建图运行后启动本节点，HBA 完成后两条 Path 发布时自动刷新显示。
 *   ros2 run automap_pro vtk_trajectory_viewer
 *
 * 依赖：VTK（find_package(VTK) 可选；未安装 VTK 时本目标不构建）。
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <mutex>
#include <vector>
#include <string>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>

#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkAxesActor.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkCamera.h>
#include <vtkCommand.h>
#include <vtkCallbackCommand.h>
#include <vtkNew.h>
#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>

namespace automap_pro {

struct TrajectoryData {
  std::vector<double> x, y, z;
  bool empty() const { return x.empty(); }
  size_t size() const { return x.size(); }
};

class VtkTrajectoryViewerNode : public rclcpp::Node {
 public:
  VtkTrajectoryViewerNode() : Node("vtk_trajectory_viewer") {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable()
        .durability(rclcpp::DurabilityPolicy::TransientLocal);  // 与发布端一致，启动后能收到 HBA 已发布的最后一次 Path
    hba_path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "/automap/optimized_path", qos,
        [this](const nav_msgs::msg::Path::SharedPtr msg) { onHbaPath(msg); });
    gps_path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "/automap/gps_keyframe_path", qos,
        [this](const nav_msgs::msg::Path::SharedPtr msg) { onGpsPath(msg); });

    RCLCPP_INFO(get_logger(), "[VtkTrajectoryViewer] Subscribed to /automap/optimized_path, /automap/gps_keyframe_path");
  }

  void onHbaPath(const nav_msgs::msg::Path::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    hba_.x.clear(); hba_.y.clear(); hba_.z.clear();
    for (const auto& p : msg->poses) {
      hba_.x.push_back(p.pose.position.x);
      hba_.y.push_back(p.pose.position.y);
      hba_.z.push_back(p.pose.position.z);
    }
    data_updated_ = true;
  }

  void onGpsPath(const nav_msgs::msg::Path::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    gps_.x.clear(); gps_.y.clear(); gps_.z.clear();
    for (const auto& p : msg->poses) {
      gps_.x.push_back(p.pose.position.x);
      gps_.y.push_back(p.pose.position.y);
      gps_.z.push_back(p.pose.position.z);
    }
    data_updated_ = true;
  }

  bool pullData(TrajectoryData* hba_out, TrajectoryData* gps_out,
                std::vector<double>* deviation_out, std::vector<double>* cum_dist_out) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!data_updated_ || hba_.empty() || gps_.size() != hba_.size())
      return false;
    *hba_out = hba_;
    *gps_out = gps_;
    deviation_out->resize(hba_.size());
    cum_dist_out->resize(hba_.size());
    double cum = 0.0;
    for (size_t i = 0; i < hba_.size(); ++i) {
      double dx = hba_.x[i] - gps_.x[i];
      double dy = hba_.y[i] - gps_.y[i];
      double dz = hba_.z[i] - gps_.z[i];
      (*deviation_out)[i] = std::sqrt(dx*dx + dy*dy + dz*dz);
      if (i > 0) {
        double ds = std::sqrt(
            std::pow(hba_.x[i]-hba_.x[i-1],2) +
            std::pow(hba_.y[i]-hba_.y[i-1],2) +
            std::pow(hba_.z[i]-hba_.z[i-1],2));
        cum += ds;
      }
      (*cum_dist_out)[i] = cum;
    }
    data_updated_ = false;
    return true;
  }

  bool hasUpdate() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return data_updated_;
  }

 private:
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr hba_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr gps_path_sub_;
  mutable std::mutex mutex_;
  TrajectoryData hba_, gps_;
  bool data_updated_ = false;
};

// 由 points 构建一条折线的 vtkPolyData（单条 LINE 串）
static vtkSmartPointer<vtkPolyData> buildPolyData(
    const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& z) {
  auto points = vtkSmartPointer<vtkPoints>::New();
  for (size_t i = 0; i < x.size(); ++i)
    points->InsertNextPoint(x[i], y[i], z[i]);
  auto lines = vtkSmartPointer<vtkCellArray>::New();
  if (x.size() >= 2) {
    lines->InsertNextCell(static_cast<vtkIdType>(x.size()));
    for (vtkIdType i = 0; i < static_cast<vtkIdType>(x.size()); ++i)
      lines->InsertCellPoint(i);
  }
  auto poly = vtkSmartPointer<vtkPolyData>::New();
  poly->SetPoints(points);
  poly->SetLines(lines);
  return poly;
}

// 偏差曲线：x = 累积里程, y = 固定偏移（与轨迹分离）, z = deviation * scale
static vtkSmartPointer<vtkPolyData> buildDeviationPolyData(
    const std::vector<double>& cum_dist, const std::vector<double>& deviation,
    double scale_z, double offset_y) {
  auto points = vtkSmartPointer<vtkPoints>::New();
  for (size_t i = 0; i < cum_dist.size(); ++i)
    points->InsertNextPoint(cum_dist[i], offset_y, deviation[i] * scale_z);
  auto lines = vtkSmartPointer<vtkCellArray>::New();
  if (cum_dist.size() >= 2) {
    lines->InsertNextCell(static_cast<vtkIdType>(cum_dist.size()));
    for (vtkIdType i = 0; i < static_cast<vtkIdType>(cum_dist.size()); ++i)
      lines->InsertCellPoint(i);
  }
  auto poly = vtkSmartPointer<vtkPolyData>::New();
  poly->SetPoints(points);
  poly->SetLines(lines);
  return poly;
}

static void setActorColor(vtkActor* actor, double r, double g, double b) {
  actor->GetProperty()->SetColor(r, g, b);
  actor->GetProperty()->SetLineWidth(2.0);
}

}  // namespace automap_pro

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<automap_pro::VtkTrajectoryViewerNode>();

  vtkNew<vtkRenderer> renderer;
  renderer->SetBackground(0.1, 0.1, 0.15);

  vtkNew<vtkRenderWindow> renWin;
  renWin->AddRenderer(renderer);
  renWin->SetSize(1200, 800);
  renWin->SetWindowName("HBA vs GPS Trajectory (VTK)");

  vtkNew<vtkRenderWindowInteractor> iren;
  iren->SetRenderWindow(renWin);

  // 三个曲线 Actor（数据稍后更新）
  vtkNew<vtkPolyData> emptyPoly;
  vtkNew<vtkPoints> emptyPts;
  emptyPts->InsertNextPoint(0, 0, 0);
  emptyPoly->SetPoints(emptyPts);

  vtkNew<vtkPolyDataMapper> mapperHba;
  mapperHba->SetInputData(emptyPoly);
  vtkNew<vtkActor> actorHba;
  actorHba->SetMapper(mapperHba);
  automap_pro::setActorColor(actorHba, 0.0, 1.0, 0.0);  // 绿 HBA
  renderer->AddActor(actorHba);

  vtkNew<vtkPolyDataMapper> mapperGps;
  mapperGps->SetInputData(emptyPoly);
  vtkNew<vtkActor> actorGps;
  actorGps->SetMapper(mapperGps);
  automap_pro::setActorColor(actorGps, 0.2, 0.4, 1.0);  // 蓝 GPS
  renderer->AddActor(actorGps);

  vtkNew<vtkPolyDataMapper> mapperDev;
  mapperDev->SetInputData(emptyPoly);
  vtkNew<vtkActor> actorDev;
  actorDev->SetMapper(mapperDev);
  automap_pro::setActorColor(actorDev, 1.0, 0.3, 0.3);  // 红 偏差曲线
  renderer->AddActor(actorDev);

  // 文字说明
  vtkNew<vtkTextActor> textActor;
  textActor->SetInput("Green: HBA trajectory  |  Blue: GPS trajectory  |  Red: Deviation (m) vs distance");
  textActor->SetPosition(10, 10);
  textActor->GetTextProperty()->SetFontSize(14);
  textActor->GetTextProperty()->SetColor(1.0, 1.0, 1.0);
  renderer->AddActor2D(textActor);

  // 定时器：处理 ROS 并刷新几何。VTK 9.x 的 AddObserver 不接受 lambda，需用 vtkCallbackCommand。
  // node 须为 VtkTrajectoryViewerNode* 以便在回调中调用 pullData（基类 rclcpp::Node 无此方法）
  const char* save_dir_env = std::getenv("AUTOMAP_ACCURACY_SAVE_DIR");
  std::string accuracy_save_dir = save_dir_env ? std::string(save_dir_env) : std::string();

  struct TimerClientData {
    std::shared_ptr<automap_pro::VtkTrajectoryViewerNode> node;
    vtkPolyDataMapper* mapperHba;
    vtkPolyDataMapper* mapperGps;
    vtkPolyDataMapper* mapperDev;
    vtkRenderWindow* renWin;
    vtkRenderer* renderer = nullptr;  // 首次有数据后 ResetCamera 用
    bool first_success = true;
    std::string save_dir;   // 精度图保存目录（来自环境变量）
    bool screenshot_saved = false;
  } client_data = { node, mapperHba, mapperGps, mapperDev, renWin, renderer, true, accuracy_save_dir, false };

  auto timer_callback = [](vtkObject*, unsigned long, void* clientData, void*) {
    auto* ctx = static_cast<TimerClientData*>(clientData);
    rclcpp::spin_some(ctx->node);
    automap_pro::TrajectoryData hba, gps;
    std::vector<double> deviation, cum_dist;
    if (!ctx->node->pullData(&hba, &gps, &deviation, &cum_dist))
      return;

    auto polyHba = automap_pro::buildPolyData(hba.x, hba.y, hba.z);
    auto polyGps = automap_pro::buildPolyData(gps.x, gps.y, gps.z);

    double maxDev = 0.0;
    for (double d : deviation) if (d > maxDev) maxDev = d;
    double scaleZ = (maxDev < 1e-6) ? 1.0 : (20.0 / maxDev);
    double offsetY = -30.0;
    auto polyDev = automap_pro::buildDeviationPolyData(cum_dist, deviation, scaleZ, offsetY);

    ctx->mapperHba->SetInputData(polyHba);
    ctx->mapperGps->SetInputData(polyGps);
    ctx->mapperDev->SetInputData(polyDev);
    ctx->mapperHba->Update();
    ctx->mapperGps->Update();
    ctx->mapperDev->Update();
    if (ctx->first_success && ctx->renderer) {
      ctx->renderer->ResetCamera();
      ctx->first_success = false;
    }
    ctx->renWin->Render();

    // 若设置了保存目录且尚未保存：保存高清 PNG + 曲线数据 CSV/摘要，便于建图后查看与绘图
    if (!ctx->save_dir.empty() && !ctx->screenshot_saved) {
      ctx->screenshot_saved = true;
      const std::string& d = ctx->save_dir;

      // 1) 高清图
      std::string png_path = d + "/accuracy_curves.png";
      vtkNew<vtkWindowToImageFilter> w2i;
      w2i->SetInput(ctx->renWin);
      w2i->SetScale(2);
      w2i->SetInputBufferTypeToRGBA();
      w2i->ReadFrontBufferOff();
      w2i->Update();
      vtkNew<vtkPNGWriter> writer;
      writer->SetFileName(png_path.c_str());
      writer->SetInputConnection(w2i->GetOutputPort());
      writer->Write();
      RCLCPP_INFO(ctx->node->get_logger(), "[VtkTrajectoryViewer] saved %s", png_path.c_str());

      // 2) 完整轨迹与偏差表（与图中曲线一一对应，方便 Excel/Python 查看与绘图）
      std::string traj_csv = d + "/accuracy_trajectories.csv";
      std::ofstream ot(traj_csv);
      if (ot.is_open()) {
        ot << "index,hba_x_m,hba_y_m,hba_z_m,gps_x_m,gps_y_m,gps_z_m,deviation_m,cum_dist_m\n";
        ot << std::fixed << std::setprecision(6);
        for (size_t i = 0; i < hba.x.size(); ++i)
          ot << i << "," << hba.x[i] << "," << hba.y[i] << "," << hba.z[i] << ","
             << gps.x[i] << "," << gps.y[i] << "," << gps.z[i] << ","
             << deviation[i] << "," << cum_dist[i] << "\n";
        ot.close();
        RCLCPP_INFO(ctx->node->get_logger(), "[VtkTrajectoryViewer] saved %s (%zu points)", traj_csv.c_str(), hba.x.size());
      }

      // 3) 偏差曲线两列（横轴累积里程、纵轴偏差，便于直接做 2D 图）
      std::string dev_csv = d + "/deviation_curve.csv";
      std::ofstream od(dev_csv);
      if (od.is_open()) {
        od << "cum_dist_m,deviation_m\n";
        od << std::fixed << std::setprecision(6);
        for (size_t i = 0; i < cum_dist.size(); ++i)
          od << cum_dist[i] << "," << deviation[i] << "\n";
        od.close();
        RCLCPP_INFO(ctx->node->get_logger(), "[VtkTrajectoryViewer] saved %s", dev_csv.c_str());
      }

      // 4) 精度摘要（便于快速查看）
      double mean_dev = 0.0, max_dev = 0.0;
      size_t n = deviation.size();
      for (size_t i = 0; i < n; ++i) {
        mean_dev += deviation[i];
        if (deviation[i] > max_dev) max_dev = deviation[i];
      }
      if (n > 0) mean_dev /= static_cast<double>(n);
      std::string sum_path = d + "/accuracy_summary.txt";
      std::ofstream os(sum_path);
      if (os.is_open()) {
        os << std::fixed << std::setprecision(4);
        os << "# HBA vs GPS accuracy (same data as accuracy_curves.png)\n";
        os << "keyframe_count=" << n << "\n";
        os << "mean_deviation_m=" << mean_dev << "\n";
        os << "max_deviation_m=" << max_dev << "\n";
        os << "cum_dist_total_m=" << (cum_dist.empty() ? 0.0 : cum_dist.back()) << "\n";
        os.close();
        RCLCPP_INFO(ctx->node->get_logger(), "[VtkTrajectoryViewer] saved %s (mean=%.3f m max=%.3f m)", sum_path.c_str(), mean_dev, max_dev);
      }
    }
  };

  vtkNew<vtkCallbackCommand> cmd;
  cmd->SetCallback(timer_callback);
  cmd->SetClientData(&client_data);
  iren->CreateRepeatingTimer(200);  // 200ms
  iren->AddObserver(vtkCommand::TimerEvent, cmd);

  renderer->ResetCamera();
  renWin->Render();
  iren->Start();

  rclcpp::shutdown();
  return 0;
}
