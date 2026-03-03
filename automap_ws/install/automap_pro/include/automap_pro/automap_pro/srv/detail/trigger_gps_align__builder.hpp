// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from automap_pro:srv/TriggerGpsAlign.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__SRV__DETAIL__TRIGGER_GPS_ALIGN__BUILDER_HPP_
#define AUTOMAP_PRO__SRV__DETAIL__TRIGGER_GPS_ALIGN__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "automap_pro/srv/detail/trigger_gps_align__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace automap_pro
{

namespace srv
{

namespace builder
{

class Init_TriggerGpsAlign_Request_force
{
public:
  Init_TriggerGpsAlign_Request_force()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::automap_pro::srv::TriggerGpsAlign_Request force(::automap_pro::srv::TriggerGpsAlign_Request::_force_type arg)
  {
    msg_.force = std::move(arg);
    return std::move(msg_);
  }

private:
  ::automap_pro::srv::TriggerGpsAlign_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::automap_pro::srv::TriggerGpsAlign_Request>()
{
  return automap_pro::srv::builder::Init_TriggerGpsAlign_Request_force();
}

}  // namespace automap_pro


namespace automap_pro
{

namespace srv
{

namespace builder
{

class Init_TriggerGpsAlign_Response_message
{
public:
  explicit Init_TriggerGpsAlign_Response_message(::automap_pro::srv::TriggerGpsAlign_Response & msg)
  : msg_(msg)
  {}
  ::automap_pro::srv::TriggerGpsAlign_Response message(::automap_pro::srv::TriggerGpsAlign_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::automap_pro::srv::TriggerGpsAlign_Response msg_;
};

class Init_TriggerGpsAlign_Response_t_gps_lidar
{
public:
  explicit Init_TriggerGpsAlign_Response_t_gps_lidar(::automap_pro::srv::TriggerGpsAlign_Response & msg)
  : msg_(msg)
  {}
  Init_TriggerGpsAlign_Response_message t_gps_lidar(::automap_pro::srv::TriggerGpsAlign_Response::_t_gps_lidar_type arg)
  {
    msg_.t_gps_lidar = std::move(arg);
    return Init_TriggerGpsAlign_Response_message(msg_);
  }

private:
  ::automap_pro::srv::TriggerGpsAlign_Response msg_;
};

class Init_TriggerGpsAlign_Response_r_gps_lidar
{
public:
  explicit Init_TriggerGpsAlign_Response_r_gps_lidar(::automap_pro::srv::TriggerGpsAlign_Response & msg)
  : msg_(msg)
  {}
  Init_TriggerGpsAlign_Response_t_gps_lidar r_gps_lidar(::automap_pro::srv::TriggerGpsAlign_Response::_r_gps_lidar_type arg)
  {
    msg_.r_gps_lidar = std::move(arg);
    return Init_TriggerGpsAlign_Response_t_gps_lidar(msg_);
  }

private:
  ::automap_pro::srv::TriggerGpsAlign_Response msg_;
};

class Init_TriggerGpsAlign_Response_alignment_rmse_m
{
public:
  explicit Init_TriggerGpsAlign_Response_alignment_rmse_m(::automap_pro::srv::TriggerGpsAlign_Response & msg)
  : msg_(msg)
  {}
  Init_TriggerGpsAlign_Response_r_gps_lidar alignment_rmse_m(::automap_pro::srv::TriggerGpsAlign_Response::_alignment_rmse_m_type arg)
  {
    msg_.alignment_rmse_m = std::move(arg);
    return Init_TriggerGpsAlign_Response_r_gps_lidar(msg_);
  }

private:
  ::automap_pro::srv::TriggerGpsAlign_Response msg_;
};

class Init_TriggerGpsAlign_Response_success
{
public:
  Init_TriggerGpsAlign_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TriggerGpsAlign_Response_alignment_rmse_m success(::automap_pro::srv::TriggerGpsAlign_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_TriggerGpsAlign_Response_alignment_rmse_m(msg_);
  }

private:
  ::automap_pro::srv::TriggerGpsAlign_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::automap_pro::srv::TriggerGpsAlign_Response>()
{
  return automap_pro::srv::builder::Init_TriggerGpsAlign_Response_success();
}

}  // namespace automap_pro

#endif  // AUTOMAP_PRO__SRV__DETAIL__TRIGGER_GPS_ALIGN__BUILDER_HPP_
