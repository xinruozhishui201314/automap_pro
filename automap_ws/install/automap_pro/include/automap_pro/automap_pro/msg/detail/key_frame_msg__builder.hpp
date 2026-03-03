// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from automap_pro:msg/KeyFrameMsg.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__MSG__DETAIL__KEY_FRAME_MSG__BUILDER_HPP_
#define AUTOMAP_PRO__MSG__DETAIL__KEY_FRAME_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "automap_pro/msg/detail/key_frame_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace automap_pro
{

namespace msg
{

namespace builder
{

class Init_KeyFrameMsg_gps
{
public:
  explicit Init_KeyFrameMsg_gps(::automap_pro::msg::KeyFrameMsg & msg)
  : msg_(msg)
  {}
  ::automap_pro::msg::KeyFrameMsg gps(::automap_pro::msg::KeyFrameMsg::_gps_type arg)
  {
    msg_.gps = std::move(arg);
    return std::move(msg_);
  }

private:
  ::automap_pro::msg::KeyFrameMsg msg_;
};

class Init_KeyFrameMsg_has_gps
{
public:
  explicit Init_KeyFrameMsg_has_gps(::automap_pro::msg::KeyFrameMsg & msg)
  : msg_(msg)
  {}
  Init_KeyFrameMsg_gps has_gps(::automap_pro::msg::KeyFrameMsg::_has_gps_type arg)
  {
    msg_.has_gps = std::move(arg);
    return Init_KeyFrameMsg_gps(msg_);
  }

private:
  ::automap_pro::msg::KeyFrameMsg msg_;
};

class Init_KeyFrameMsg_cloud
{
public:
  explicit Init_KeyFrameMsg_cloud(::automap_pro::msg::KeyFrameMsg & msg)
  : msg_(msg)
  {}
  Init_KeyFrameMsg_has_gps cloud(::automap_pro::msg::KeyFrameMsg::_cloud_type arg)
  {
    msg_.cloud = std::move(arg);
    return Init_KeyFrameMsg_has_gps(msg_);
  }

private:
  ::automap_pro::msg::KeyFrameMsg msg_;
};

class Init_KeyFrameMsg_pose
{
public:
  explicit Init_KeyFrameMsg_pose(::automap_pro::msg::KeyFrameMsg & msg)
  : msg_(msg)
  {}
  Init_KeyFrameMsg_cloud pose(::automap_pro::msg::KeyFrameMsg::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_KeyFrameMsg_cloud(msg_);
  }

private:
  ::automap_pro::msg::KeyFrameMsg msg_;
};

class Init_KeyFrameMsg_submap_id
{
public:
  explicit Init_KeyFrameMsg_submap_id(::automap_pro::msg::KeyFrameMsg & msg)
  : msg_(msg)
  {}
  Init_KeyFrameMsg_pose submap_id(::automap_pro::msg::KeyFrameMsg::_submap_id_type arg)
  {
    msg_.submap_id = std::move(arg);
    return Init_KeyFrameMsg_pose(msg_);
  }

private:
  ::automap_pro::msg::KeyFrameMsg msg_;
};

class Init_KeyFrameMsg_session_id
{
public:
  explicit Init_KeyFrameMsg_session_id(::automap_pro::msg::KeyFrameMsg & msg)
  : msg_(msg)
  {}
  Init_KeyFrameMsg_submap_id session_id(::automap_pro::msg::KeyFrameMsg::_session_id_type arg)
  {
    msg_.session_id = std::move(arg);
    return Init_KeyFrameMsg_submap_id(msg_);
  }

private:
  ::automap_pro::msg::KeyFrameMsg msg_;
};

class Init_KeyFrameMsg_id
{
public:
  explicit Init_KeyFrameMsg_id(::automap_pro::msg::KeyFrameMsg & msg)
  : msg_(msg)
  {}
  Init_KeyFrameMsg_session_id id(::automap_pro::msg::KeyFrameMsg::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_KeyFrameMsg_session_id(msg_);
  }

private:
  ::automap_pro::msg::KeyFrameMsg msg_;
};

class Init_KeyFrameMsg_header
{
public:
  Init_KeyFrameMsg_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_KeyFrameMsg_id header(::automap_pro::msg::KeyFrameMsg::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_KeyFrameMsg_id(msg_);
  }

private:
  ::automap_pro::msg::KeyFrameMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::automap_pro::msg::KeyFrameMsg>()
{
  return automap_pro::msg::builder::Init_KeyFrameMsg_header();
}

}  // namespace automap_pro

#endif  // AUTOMAP_PRO__MSG__DETAIL__KEY_FRAME_MSG__BUILDER_HPP_
