// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from automap_pro:srv/SaveMap.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__SRV__DETAIL__SAVE_MAP__BUILDER_HPP_
#define AUTOMAP_PRO__SRV__DETAIL__SAVE_MAP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "automap_pro/srv/detail/save_map__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace automap_pro
{

namespace srv
{

namespace builder
{

class Init_SaveMap_Request_save_trajectory_kitti
{
public:
  explicit Init_SaveMap_Request_save_trajectory_kitti(::automap_pro::srv::SaveMap_Request & msg)
  : msg_(msg)
  {}
  ::automap_pro::srv::SaveMap_Request save_trajectory_kitti(::automap_pro::srv::SaveMap_Request::_save_trajectory_kitti_type arg)
  {
    msg_.save_trajectory_kitti = std::move(arg);
    return std::move(msg_);
  }

private:
  ::automap_pro::srv::SaveMap_Request msg_;
};

class Init_SaveMap_Request_save_trajectory_tum
{
public:
  explicit Init_SaveMap_Request_save_trajectory_tum(::automap_pro::srv::SaveMap_Request & msg)
  : msg_(msg)
  {}
  Init_SaveMap_Request_save_trajectory_kitti save_trajectory_tum(::automap_pro::srv::SaveMap_Request::_save_trajectory_tum_type arg)
  {
    msg_.save_trajectory_tum = std::move(arg);
    return Init_SaveMap_Request_save_trajectory_kitti(msg_);
  }

private:
  ::automap_pro::srv::SaveMap_Request msg_;
};

class Init_SaveMap_Request_save_las
{
public:
  explicit Init_SaveMap_Request_save_las(::automap_pro::srv::SaveMap_Request & msg)
  : msg_(msg)
  {}
  Init_SaveMap_Request_save_trajectory_tum save_las(::automap_pro::srv::SaveMap_Request::_save_las_type arg)
  {
    msg_.save_las = std::move(arg);
    return Init_SaveMap_Request_save_trajectory_tum(msg_);
  }

private:
  ::automap_pro::srv::SaveMap_Request msg_;
};

class Init_SaveMap_Request_save_ply
{
public:
  explicit Init_SaveMap_Request_save_ply(::automap_pro::srv::SaveMap_Request & msg)
  : msg_(msg)
  {}
  Init_SaveMap_Request_save_las save_ply(::automap_pro::srv::SaveMap_Request::_save_ply_type arg)
  {
    msg_.save_ply = std::move(arg);
    return Init_SaveMap_Request_save_las(msg_);
  }

private:
  ::automap_pro::srv::SaveMap_Request msg_;
};

class Init_SaveMap_Request_save_pcd
{
public:
  explicit Init_SaveMap_Request_save_pcd(::automap_pro::srv::SaveMap_Request & msg)
  : msg_(msg)
  {}
  Init_SaveMap_Request_save_ply save_pcd(::automap_pro::srv::SaveMap_Request::_save_pcd_type arg)
  {
    msg_.save_pcd = std::move(arg);
    return Init_SaveMap_Request_save_ply(msg_);
  }

private:
  ::automap_pro::srv::SaveMap_Request msg_;
};

class Init_SaveMap_Request_output_dir
{
public:
  Init_SaveMap_Request_output_dir()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SaveMap_Request_save_pcd output_dir(::automap_pro::srv::SaveMap_Request::_output_dir_type arg)
  {
    msg_.output_dir = std::move(arg);
    return Init_SaveMap_Request_save_pcd(msg_);
  }

private:
  ::automap_pro::srv::SaveMap_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::automap_pro::srv::SaveMap_Request>()
{
  return automap_pro::srv::builder::Init_SaveMap_Request_output_dir();
}

}  // namespace automap_pro


namespace automap_pro
{

namespace srv
{

namespace builder
{

class Init_SaveMap_Response_message
{
public:
  explicit Init_SaveMap_Response_message(::automap_pro::srv::SaveMap_Response & msg)
  : msg_(msg)
  {}
  ::automap_pro::srv::SaveMap_Response message(::automap_pro::srv::SaveMap_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::automap_pro::srv::SaveMap_Response msg_;
};

class Init_SaveMap_Response_output_path
{
public:
  explicit Init_SaveMap_Response_output_path(::automap_pro::srv::SaveMap_Response & msg)
  : msg_(msg)
  {}
  Init_SaveMap_Response_message output_path(::automap_pro::srv::SaveMap_Response::_output_path_type arg)
  {
    msg_.output_path = std::move(arg);
    return Init_SaveMap_Response_message(msg_);
  }

private:
  ::automap_pro::srv::SaveMap_Response msg_;
};

class Init_SaveMap_Response_success
{
public:
  Init_SaveMap_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SaveMap_Response_output_path success(::automap_pro::srv::SaveMap_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SaveMap_Response_output_path(msg_);
  }

private:
  ::automap_pro::srv::SaveMap_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::automap_pro::srv::SaveMap_Response>()
{
  return automap_pro::srv::builder::Init_SaveMap_Response_success();
}

}  // namespace automap_pro

#endif  // AUTOMAP_PRO__SRV__DETAIL__SAVE_MAP__BUILDER_HPP_
