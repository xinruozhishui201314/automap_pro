// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from automap_pro:srv/GetStatus.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__SRV__DETAIL__GET_STATUS__BUILDER_HPP_
#define AUTOMAP_PRO__SRV__DETAIL__GET_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "automap_pro/srv/detail/get_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace automap_pro
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::automap_pro::srv::GetStatus_Request>()
{
  return ::automap_pro::srv::GetStatus_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace automap_pro


namespace automap_pro
{

namespace srv
{

namespace builder
{

class Init_GetStatus_Response_total_distance_m
{
public:
  explicit Init_GetStatus_Response_total_distance_m(::automap_pro::srv::GetStatus_Response & msg)
  : msg_(msg)
  {}
  ::automap_pro::srv::GetStatus_Response total_distance_m(::automap_pro::srv::GetStatus_Response::_total_distance_m_type arg)
  {
    msg_.total_distance_m = std::move(arg);
    return std::move(msg_);
  }

private:
  ::automap_pro::srv::GetStatus_Response msg_;
};

class Init_GetStatus_Response_gps_align_score
{
public:
  explicit Init_GetStatus_Response_gps_align_score(::automap_pro::srv::GetStatus_Response & msg)
  : msg_(msg)
  {}
  Init_GetStatus_Response_total_distance_m gps_align_score(::automap_pro::srv::GetStatus_Response::_gps_align_score_type arg)
  {
    msg_.gps_align_score = std::move(arg);
    return Init_GetStatus_Response_total_distance_m(msg_);
  }

private:
  ::automap_pro::srv::GetStatus_Response msg_;
};

class Init_GetStatus_Response_gps_aligned
{
public:
  explicit Init_GetStatus_Response_gps_aligned(::automap_pro::srv::GetStatus_Response & msg)
  : msg_(msg)
  {}
  Init_GetStatus_Response_gps_align_score gps_aligned(::automap_pro::srv::GetStatus_Response::_gps_aligned_type arg)
  {
    msg_.gps_aligned = std::move(arg);
    return Init_GetStatus_Response_gps_align_score(msg_);
  }

private:
  ::automap_pro::srv::GetStatus_Response msg_;
};

class Init_GetStatus_Response_loop_count
{
public:
  explicit Init_GetStatus_Response_loop_count(::automap_pro::srv::GetStatus_Response & msg)
  : msg_(msg)
  {}
  Init_GetStatus_Response_gps_aligned loop_count(::automap_pro::srv::GetStatus_Response::_loop_count_type arg)
  {
    msg_.loop_count = std::move(arg);
    return Init_GetStatus_Response_gps_aligned(msg_);
  }

private:
  ::automap_pro::srv::GetStatus_Response msg_;
};

class Init_GetStatus_Response_submap_count
{
public:
  explicit Init_GetStatus_Response_submap_count(::automap_pro::srv::GetStatus_Response & msg)
  : msg_(msg)
  {}
  Init_GetStatus_Response_loop_count submap_count(::automap_pro::srv::GetStatus_Response::_submap_count_type arg)
  {
    msg_.submap_count = std::move(arg);
    return Init_GetStatus_Response_loop_count(msg_);
  }

private:
  ::automap_pro::srv::GetStatus_Response msg_;
};

class Init_GetStatus_Response_keyframe_count
{
public:
  explicit Init_GetStatus_Response_keyframe_count(::automap_pro::srv::GetStatus_Response & msg)
  : msg_(msg)
  {}
  Init_GetStatus_Response_submap_count keyframe_count(::automap_pro::srv::GetStatus_Response::_keyframe_count_type arg)
  {
    msg_.keyframe_count = std::move(arg);
    return Init_GetStatus_Response_submap_count(msg_);
  }

private:
  ::automap_pro::srv::GetStatus_Response msg_;
};

class Init_GetStatus_Response_session_id
{
public:
  explicit Init_GetStatus_Response_session_id(::automap_pro::srv::GetStatus_Response & msg)
  : msg_(msg)
  {}
  Init_GetStatus_Response_keyframe_count session_id(::automap_pro::srv::GetStatus_Response::_session_id_type arg)
  {
    msg_.session_id = std::move(arg);
    return Init_GetStatus_Response_keyframe_count(msg_);
  }

private:
  ::automap_pro::srv::GetStatus_Response msg_;
};

class Init_GetStatus_Response_state
{
public:
  Init_GetStatus_Response_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetStatus_Response_session_id state(::automap_pro::srv::GetStatus_Response::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_GetStatus_Response_session_id(msg_);
  }

private:
  ::automap_pro::srv::GetStatus_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::automap_pro::srv::GetStatus_Response>()
{
  return automap_pro::srv::builder::Init_GetStatus_Response_state();
}

}  // namespace automap_pro

#endif  // AUTOMAP_PRO__SRV__DETAIL__GET_STATUS__BUILDER_HPP_
