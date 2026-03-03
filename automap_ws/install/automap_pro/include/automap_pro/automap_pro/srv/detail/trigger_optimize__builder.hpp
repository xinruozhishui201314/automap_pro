// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from automap_pro:srv/TriggerOptimize.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__SRV__DETAIL__TRIGGER_OPTIMIZE__BUILDER_HPP_
#define AUTOMAP_PRO__SRV__DETAIL__TRIGGER_OPTIMIZE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "automap_pro/srv/detail/trigger_optimize__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace automap_pro
{

namespace srv
{

namespace builder
{

class Init_TriggerOptimize_Request_max_iterations
{
public:
  explicit Init_TriggerOptimize_Request_max_iterations(::automap_pro::srv::TriggerOptimize_Request & msg)
  : msg_(msg)
  {}
  ::automap_pro::srv::TriggerOptimize_Request max_iterations(::automap_pro::srv::TriggerOptimize_Request::_max_iterations_type arg)
  {
    msg_.max_iterations = std::move(arg);
    return std::move(msg_);
  }

private:
  ::automap_pro::srv::TriggerOptimize_Request msg_;
};

class Init_TriggerOptimize_Request_full_optimization
{
public:
  Init_TriggerOptimize_Request_full_optimization()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TriggerOptimize_Request_max_iterations full_optimization(::automap_pro::srv::TriggerOptimize_Request::_full_optimization_type arg)
  {
    msg_.full_optimization = std::move(arg);
    return Init_TriggerOptimize_Request_max_iterations(msg_);
  }

private:
  ::automap_pro::srv::TriggerOptimize_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::automap_pro::srv::TriggerOptimize_Request>()
{
  return automap_pro::srv::builder::Init_TriggerOptimize_Request_full_optimization();
}

}  // namespace automap_pro


namespace automap_pro
{

namespace srv
{

namespace builder
{

class Init_TriggerOptimize_Response_nodes_updated
{
public:
  explicit Init_TriggerOptimize_Response_nodes_updated(::automap_pro::srv::TriggerOptimize_Response & msg)
  : msg_(msg)
  {}
  ::automap_pro::srv::TriggerOptimize_Response nodes_updated(::automap_pro::srv::TriggerOptimize_Response::_nodes_updated_type arg)
  {
    msg_.nodes_updated = std::move(arg);
    return std::move(msg_);
  }

private:
  ::automap_pro::srv::TriggerOptimize_Response msg_;
};

class Init_TriggerOptimize_Response_elapsed_seconds
{
public:
  explicit Init_TriggerOptimize_Response_elapsed_seconds(::automap_pro::srv::TriggerOptimize_Response & msg)
  : msg_(msg)
  {}
  Init_TriggerOptimize_Response_nodes_updated elapsed_seconds(::automap_pro::srv::TriggerOptimize_Response::_elapsed_seconds_type arg)
  {
    msg_.elapsed_seconds = std::move(arg);
    return Init_TriggerOptimize_Response_nodes_updated(msg_);
  }

private:
  ::automap_pro::srv::TriggerOptimize_Response msg_;
};

class Init_TriggerOptimize_Response_success
{
public:
  Init_TriggerOptimize_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TriggerOptimize_Response_elapsed_seconds success(::automap_pro::srv::TriggerOptimize_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_TriggerOptimize_Response_elapsed_seconds(msg_);
  }

private:
  ::automap_pro::srv::TriggerOptimize_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::automap_pro::srv::TriggerOptimize_Response>()
{
  return automap_pro::srv::builder::Init_TriggerOptimize_Response_success();
}

}  // namespace automap_pro

#endif  // AUTOMAP_PRO__SRV__DETAIL__TRIGGER_OPTIMIZE__BUILDER_HPP_
