// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from automap_pro:msg/SubMapEventMsg.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__MSG__DETAIL__SUB_MAP_EVENT_MSG__BUILDER_HPP_
#define AUTOMAP_PRO__MSG__DETAIL__SUB_MAP_EVENT_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "automap_pro/msg/detail/sub_map_event_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace automap_pro
{

namespace msg
{

namespace builder
{

class Init_SubMapEventMsg_has_valid_gps
{
public:
  explicit Init_SubMapEventMsg_has_valid_gps(::automap_pro::msg::SubMapEventMsg & msg)
  : msg_(msg)
  {}
  ::automap_pro::msg::SubMapEventMsg has_valid_gps(::automap_pro::msg::SubMapEventMsg::_has_valid_gps_type arg)
  {
    msg_.has_valid_gps = std::move(arg);
    return std::move(msg_);
  }

private:
  ::automap_pro::msg::SubMapEventMsg msg_;
};

class Init_SubMapEventMsg_anchor_pose
{
public:
  explicit Init_SubMapEventMsg_anchor_pose(::automap_pro::msg::SubMapEventMsg & msg)
  : msg_(msg)
  {}
  Init_SubMapEventMsg_has_valid_gps anchor_pose(::automap_pro::msg::SubMapEventMsg::_anchor_pose_type arg)
  {
    msg_.anchor_pose = std::move(arg);
    return Init_SubMapEventMsg_has_valid_gps(msg_);
  }

private:
  ::automap_pro::msg::SubMapEventMsg msg_;
};

class Init_SubMapEventMsg_spatial_extent_m
{
public:
  explicit Init_SubMapEventMsg_spatial_extent_m(::automap_pro::msg::SubMapEventMsg & msg)
  : msg_(msg)
  {}
  Init_SubMapEventMsg_anchor_pose spatial_extent_m(::automap_pro::msg::SubMapEventMsg::_spatial_extent_m_type arg)
  {
    msg_.spatial_extent_m = std::move(arg);
    return Init_SubMapEventMsg_anchor_pose(msg_);
  }

private:
  ::automap_pro::msg::SubMapEventMsg msg_;
};

class Init_SubMapEventMsg_keyframe_count
{
public:
  explicit Init_SubMapEventMsg_keyframe_count(::automap_pro::msg::SubMapEventMsg & msg)
  : msg_(msg)
  {}
  Init_SubMapEventMsg_spatial_extent_m keyframe_count(::automap_pro::msg::SubMapEventMsg::_keyframe_count_type arg)
  {
    msg_.keyframe_count = std::move(arg);
    return Init_SubMapEventMsg_spatial_extent_m(msg_);
  }

private:
  ::automap_pro::msg::SubMapEventMsg msg_;
};

class Init_SubMapEventMsg_event_type
{
public:
  explicit Init_SubMapEventMsg_event_type(::automap_pro::msg::SubMapEventMsg & msg)
  : msg_(msg)
  {}
  Init_SubMapEventMsg_keyframe_count event_type(::automap_pro::msg::SubMapEventMsg::_event_type_type arg)
  {
    msg_.event_type = std::move(arg);
    return Init_SubMapEventMsg_keyframe_count(msg_);
  }

private:
  ::automap_pro::msg::SubMapEventMsg msg_;
};

class Init_SubMapEventMsg_session_id
{
public:
  explicit Init_SubMapEventMsg_session_id(::automap_pro::msg::SubMapEventMsg & msg)
  : msg_(msg)
  {}
  Init_SubMapEventMsg_event_type session_id(::automap_pro::msg::SubMapEventMsg::_session_id_type arg)
  {
    msg_.session_id = std::move(arg);
    return Init_SubMapEventMsg_event_type(msg_);
  }

private:
  ::automap_pro::msg::SubMapEventMsg msg_;
};

class Init_SubMapEventMsg_submap_id
{
public:
  explicit Init_SubMapEventMsg_submap_id(::automap_pro::msg::SubMapEventMsg & msg)
  : msg_(msg)
  {}
  Init_SubMapEventMsg_session_id submap_id(::automap_pro::msg::SubMapEventMsg::_submap_id_type arg)
  {
    msg_.submap_id = std::move(arg);
    return Init_SubMapEventMsg_session_id(msg_);
  }

private:
  ::automap_pro::msg::SubMapEventMsg msg_;
};

class Init_SubMapEventMsg_header
{
public:
  Init_SubMapEventMsg_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SubMapEventMsg_submap_id header(::automap_pro::msg::SubMapEventMsg::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SubMapEventMsg_submap_id(msg_);
  }

private:
  ::automap_pro::msg::SubMapEventMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::automap_pro::msg::SubMapEventMsg>()
{
  return automap_pro::msg::builder::Init_SubMapEventMsg_header();
}

}  // namespace automap_pro

#endif  // AUTOMAP_PRO__MSG__DETAIL__SUB_MAP_EVENT_MSG__BUILDER_HPP_
