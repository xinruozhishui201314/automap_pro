// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from automap_pro:msg/MappingStatusMsg.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__MSG__DETAIL__MAPPING_STATUS_MSG__BUILDER_HPP_
#define AUTOMAP_PRO__MSG__DETAIL__MAPPING_STATUS_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "automap_pro/msg/detail/mapping_status_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace automap_pro
{

namespace msg
{

namespace builder
{

class Init_MappingStatusMsg_total_distance_m
{
public:
  explicit Init_MappingStatusMsg_total_distance_m(::automap_pro::msg::MappingStatusMsg & msg)
  : msg_(msg)
  {}
  ::automap_pro::msg::MappingStatusMsg total_distance_m(::automap_pro::msg::MappingStatusMsg::_total_distance_m_type arg)
  {
    msg_.total_distance_m = std::move(arg);
    return std::move(msg_);
  }

private:
  ::automap_pro::msg::MappingStatusMsg msg_;
};

class Init_MappingStatusMsg_map_entropy
{
public:
  explicit Init_MappingStatusMsg_map_entropy(::automap_pro::msg::MappingStatusMsg & msg)
  : msg_(msg)
  {}
  Init_MappingStatusMsg_total_distance_m map_entropy(::automap_pro::msg::MappingStatusMsg::_map_entropy_type arg)
  {
    msg_.map_entropy = std::move(arg);
    return Init_MappingStatusMsg_total_distance_m(msg_);
  }

private:
  ::automap_pro::msg::MappingStatusMsg msg_;
};

class Init_MappingStatusMsg_gps_alignment_score
{
public:
  explicit Init_MappingStatusMsg_gps_alignment_score(::automap_pro::msg::MappingStatusMsg & msg)
  : msg_(msg)
  {}
  Init_MappingStatusMsg_map_entropy gps_alignment_score(::automap_pro::msg::MappingStatusMsg::_gps_alignment_score_type arg)
  {
    msg_.gps_alignment_score = std::move(arg);
    return Init_MappingStatusMsg_map_entropy(msg_);
  }

private:
  ::automap_pro::msg::MappingStatusMsg msg_;
};

class Init_MappingStatusMsg_gps_aligned
{
public:
  explicit Init_MappingStatusMsg_gps_aligned(::automap_pro::msg::MappingStatusMsg & msg)
  : msg_(msg)
  {}
  Init_MappingStatusMsg_gps_alignment_score gps_aligned(::automap_pro::msg::MappingStatusMsg::_gps_aligned_type arg)
  {
    msg_.gps_aligned = std::move(arg);
    return Init_MappingStatusMsg_gps_alignment_score(msg_);
  }

private:
  ::automap_pro::msg::MappingStatusMsg msg_;
};

class Init_MappingStatusMsg_loop_count
{
public:
  explicit Init_MappingStatusMsg_loop_count(::automap_pro::msg::MappingStatusMsg & msg)
  : msg_(msg)
  {}
  Init_MappingStatusMsg_gps_aligned loop_count(::automap_pro::msg::MappingStatusMsg::_loop_count_type arg)
  {
    msg_.loop_count = std::move(arg);
    return Init_MappingStatusMsg_gps_aligned(msg_);
  }

private:
  ::automap_pro::msg::MappingStatusMsg msg_;
};

class Init_MappingStatusMsg_submap_count
{
public:
  explicit Init_MappingStatusMsg_submap_count(::automap_pro::msg::MappingStatusMsg & msg)
  : msg_(msg)
  {}
  Init_MappingStatusMsg_loop_count submap_count(::automap_pro::msg::MappingStatusMsg::_submap_count_type arg)
  {
    msg_.submap_count = std::move(arg);
    return Init_MappingStatusMsg_loop_count(msg_);
  }

private:
  ::automap_pro::msg::MappingStatusMsg msg_;
};

class Init_MappingStatusMsg_keyframe_count
{
public:
  explicit Init_MappingStatusMsg_keyframe_count(::automap_pro::msg::MappingStatusMsg & msg)
  : msg_(msg)
  {}
  Init_MappingStatusMsg_submap_count keyframe_count(::automap_pro::msg::MappingStatusMsg::_keyframe_count_type arg)
  {
    msg_.keyframe_count = std::move(arg);
    return Init_MappingStatusMsg_submap_count(msg_);
  }

private:
  ::automap_pro::msg::MappingStatusMsg msg_;
};

class Init_MappingStatusMsg_session_id
{
public:
  explicit Init_MappingStatusMsg_session_id(::automap_pro::msg::MappingStatusMsg & msg)
  : msg_(msg)
  {}
  Init_MappingStatusMsg_keyframe_count session_id(::automap_pro::msg::MappingStatusMsg::_session_id_type arg)
  {
    msg_.session_id = std::move(arg);
    return Init_MappingStatusMsg_keyframe_count(msg_);
  }

private:
  ::automap_pro::msg::MappingStatusMsg msg_;
};

class Init_MappingStatusMsg_state
{
public:
  explicit Init_MappingStatusMsg_state(::automap_pro::msg::MappingStatusMsg & msg)
  : msg_(msg)
  {}
  Init_MappingStatusMsg_session_id state(::automap_pro::msg::MappingStatusMsg::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_MappingStatusMsg_session_id(msg_);
  }

private:
  ::automap_pro::msg::MappingStatusMsg msg_;
};

class Init_MappingStatusMsg_header
{
public:
  Init_MappingStatusMsg_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MappingStatusMsg_state header(::automap_pro::msg::MappingStatusMsg::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MappingStatusMsg_state(msg_);
  }

private:
  ::automap_pro::msg::MappingStatusMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::automap_pro::msg::MappingStatusMsg>()
{
  return automap_pro::msg::builder::Init_MappingStatusMsg_header();
}

}  // namespace automap_pro

#endif  // AUTOMAP_PRO__MSG__DETAIL__MAPPING_STATUS_MSG__BUILDER_HPP_
