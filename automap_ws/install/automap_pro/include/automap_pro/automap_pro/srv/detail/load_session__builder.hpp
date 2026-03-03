// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from automap_pro:srv/LoadSession.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__SRV__DETAIL__LOAD_SESSION__BUILDER_HPP_
#define AUTOMAP_PRO__SRV__DETAIL__LOAD_SESSION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "automap_pro/srv/detail/load_session__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace automap_pro
{

namespace srv
{

namespace builder
{

class Init_LoadSession_Request_session_id
{
public:
  explicit Init_LoadSession_Request_session_id(::automap_pro::srv::LoadSession_Request & msg)
  : msg_(msg)
  {}
  ::automap_pro::srv::LoadSession_Request session_id(::automap_pro::srv::LoadSession_Request::_session_id_type arg)
  {
    msg_.session_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::automap_pro::srv::LoadSession_Request msg_;
};

class Init_LoadSession_Request_session_dir
{
public:
  Init_LoadSession_Request_session_dir()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LoadSession_Request_session_id session_dir(::automap_pro::srv::LoadSession_Request::_session_dir_type arg)
  {
    msg_.session_dir = std::move(arg);
    return Init_LoadSession_Request_session_id(msg_);
  }

private:
  ::automap_pro::srv::LoadSession_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::automap_pro::srv::LoadSession_Request>()
{
  return automap_pro::srv::builder::Init_LoadSession_Request_session_dir();
}

}  // namespace automap_pro


namespace automap_pro
{

namespace srv
{

namespace builder
{

class Init_LoadSession_Response_message
{
public:
  explicit Init_LoadSession_Response_message(::automap_pro::srv::LoadSession_Response & msg)
  : msg_(msg)
  {}
  ::automap_pro::srv::LoadSession_Response message(::automap_pro::srv::LoadSession_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::automap_pro::srv::LoadSession_Response msg_;
};

class Init_LoadSession_Response_descriptors_loaded
{
public:
  explicit Init_LoadSession_Response_descriptors_loaded(::automap_pro::srv::LoadSession_Response & msg)
  : msg_(msg)
  {}
  Init_LoadSession_Response_message descriptors_loaded(::automap_pro::srv::LoadSession_Response::_descriptors_loaded_type arg)
  {
    msg_.descriptors_loaded = std::move(arg);
    return Init_LoadSession_Response_message(msg_);
  }

private:
  ::automap_pro::srv::LoadSession_Response msg_;
};

class Init_LoadSession_Response_submaps_loaded
{
public:
  explicit Init_LoadSession_Response_submaps_loaded(::automap_pro::srv::LoadSession_Response & msg)
  : msg_(msg)
  {}
  Init_LoadSession_Response_descriptors_loaded submaps_loaded(::automap_pro::srv::LoadSession_Response::_submaps_loaded_type arg)
  {
    msg_.submaps_loaded = std::move(arg);
    return Init_LoadSession_Response_descriptors_loaded(msg_);
  }

private:
  ::automap_pro::srv::LoadSession_Response msg_;
};

class Init_LoadSession_Response_success
{
public:
  Init_LoadSession_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LoadSession_Response_submaps_loaded success(::automap_pro::srv::LoadSession_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_LoadSession_Response_submaps_loaded(msg_);
  }

private:
  ::automap_pro::srv::LoadSession_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::automap_pro::srv::LoadSession_Response>()
{
  return automap_pro::srv::builder::Init_LoadSession_Response_success();
}

}  // namespace automap_pro

#endif  // AUTOMAP_PRO__SRV__DETAIL__LOAD_SESSION__BUILDER_HPP_
