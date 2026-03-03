// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from automap_pro:srv/TriggerHBA.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__SRV__DETAIL__TRIGGER_HBA__BUILDER_HPP_
#define AUTOMAP_PRO__SRV__DETAIL__TRIGGER_HBA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "automap_pro/srv/detail/trigger_hba__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace automap_pro
{

namespace srv
{

namespace builder
{

class Init_TriggerHBA_Request_wait_for_result
{
public:
  Init_TriggerHBA_Request_wait_for_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::automap_pro::srv::TriggerHBA_Request wait_for_result(::automap_pro::srv::TriggerHBA_Request::_wait_for_result_type arg)
  {
    msg_.wait_for_result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::automap_pro::srv::TriggerHBA_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::automap_pro::srv::TriggerHBA_Request>()
{
  return automap_pro::srv::builder::Init_TriggerHBA_Request_wait_for_result();
}

}  // namespace automap_pro


namespace automap_pro
{

namespace srv
{

namespace builder
{

class Init_TriggerHBA_Response_final_mme
{
public:
  explicit Init_TriggerHBA_Response_final_mme(::automap_pro::srv::TriggerHBA_Response & msg)
  : msg_(msg)
  {}
  ::automap_pro::srv::TriggerHBA_Response final_mme(::automap_pro::srv::TriggerHBA_Response::_final_mme_type arg)
  {
    msg_.final_mme = std::move(arg);
    return std::move(msg_);
  }

private:
  ::automap_pro::srv::TriggerHBA_Response msg_;
};

class Init_TriggerHBA_Response_elapsed_seconds
{
public:
  explicit Init_TriggerHBA_Response_elapsed_seconds(::automap_pro::srv::TriggerHBA_Response & msg)
  : msg_(msg)
  {}
  Init_TriggerHBA_Response_final_mme elapsed_seconds(::automap_pro::srv::TriggerHBA_Response::_elapsed_seconds_type arg)
  {
    msg_.elapsed_seconds = std::move(arg);
    return Init_TriggerHBA_Response_final_mme(msg_);
  }

private:
  ::automap_pro::srv::TriggerHBA_Response msg_;
};

class Init_TriggerHBA_Response_message
{
public:
  explicit Init_TriggerHBA_Response_message(::automap_pro::srv::TriggerHBA_Response & msg)
  : msg_(msg)
  {}
  Init_TriggerHBA_Response_elapsed_seconds message(::automap_pro::srv::TriggerHBA_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_TriggerHBA_Response_elapsed_seconds(msg_);
  }

private:
  ::automap_pro::srv::TriggerHBA_Response msg_;
};

class Init_TriggerHBA_Response_success
{
public:
  Init_TriggerHBA_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TriggerHBA_Response_message success(::automap_pro::srv::TriggerHBA_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_TriggerHBA_Response_message(msg_);
  }

private:
  ::automap_pro::srv::TriggerHBA_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::automap_pro::srv::TriggerHBA_Response>()
{
  return automap_pro::srv::builder::Init_TriggerHBA_Response_success();
}

}  // namespace automap_pro

#endif  // AUTOMAP_PRO__SRV__DETAIL__TRIGGER_HBA__BUILDER_HPP_
