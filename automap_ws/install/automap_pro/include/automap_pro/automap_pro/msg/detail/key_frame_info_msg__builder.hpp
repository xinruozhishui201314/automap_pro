// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from automap_pro:msg/KeyFrameInfoMsg.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__MSG__DETAIL__KEY_FRAME_INFO_MSG__BUILDER_HPP_
#define AUTOMAP_PRO__MSG__DETAIL__KEY_FRAME_INFO_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "automap_pro/msg/detail/key_frame_info_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace automap_pro
{

namespace msg
{

namespace builder
{

class Init_KeyFrameInfoMsg_pose
{
public:
  explicit Init_KeyFrameInfoMsg_pose(::automap_pro::msg::KeyFrameInfoMsg & msg)
  : msg_(msg)
  {}
  ::automap_pro::msg::KeyFrameInfoMsg pose(::automap_pro::msg::KeyFrameInfoMsg::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::automap_pro::msg::KeyFrameInfoMsg msg_;
};

class Init_KeyFrameInfoMsg_cloud_valid
{
public:
  explicit Init_KeyFrameInfoMsg_cloud_valid(::automap_pro::msg::KeyFrameInfoMsg & msg)
  : msg_(msg)
  {}
  Init_KeyFrameInfoMsg_pose cloud_valid(::automap_pro::msg::KeyFrameInfoMsg::_cloud_valid_type arg)
  {
    msg_.cloud_valid = std::move(arg);
    return Init_KeyFrameInfoMsg_pose(msg_);
  }

private:
  ::automap_pro::msg::KeyFrameInfoMsg msg_;
};

class Init_KeyFrameInfoMsg_cloud_point_count
{
public:
  explicit Init_KeyFrameInfoMsg_cloud_point_count(::automap_pro::msg::KeyFrameInfoMsg & msg)
  : msg_(msg)
  {}
  Init_KeyFrameInfoMsg_cloud_valid cloud_point_count(::automap_pro::msg::KeyFrameInfoMsg::_cloud_point_count_type arg)
  {
    msg_.cloud_point_count = std::move(arg);
    return Init_KeyFrameInfoMsg_cloud_valid(msg_);
  }

private:
  ::automap_pro::msg::KeyFrameInfoMsg msg_;
};

class Init_KeyFrameInfoMsg_accel_bias
{
public:
  explicit Init_KeyFrameInfoMsg_accel_bias(::automap_pro::msg::KeyFrameInfoMsg & msg)
  : msg_(msg)
  {}
  Init_KeyFrameInfoMsg_cloud_point_count accel_bias(::automap_pro::msg::KeyFrameInfoMsg::_accel_bias_type arg)
  {
    msg_.accel_bias = std::move(arg);
    return Init_KeyFrameInfoMsg_cloud_point_count(msg_);
  }

private:
  ::automap_pro::msg::KeyFrameInfoMsg msg_;
};

class Init_KeyFrameInfoMsg_gyro_bias
{
public:
  explicit Init_KeyFrameInfoMsg_gyro_bias(::automap_pro::msg::KeyFrameInfoMsg & msg)
  : msg_(msg)
  {}
  Init_KeyFrameInfoMsg_accel_bias gyro_bias(::automap_pro::msg::KeyFrameInfoMsg::_gyro_bias_type arg)
  {
    msg_.gyro_bias = std::move(arg);
    return Init_KeyFrameInfoMsg_accel_bias(msg_);
  }

private:
  ::automap_pro::msg::KeyFrameInfoMsg msg_;
};

class Init_KeyFrameInfoMsg_map_update_rate
{
public:
  explicit Init_KeyFrameInfoMsg_map_update_rate(::automap_pro::msg::KeyFrameInfoMsg & msg)
  : msg_(msg)
  {}
  Init_KeyFrameInfoMsg_gyro_bias map_update_rate(::automap_pro::msg::KeyFrameInfoMsg::_map_update_rate_type arg)
  {
    msg_.map_update_rate = std::move(arg);
    return Init_KeyFrameInfoMsg_gyro_bias(msg_);
  }

private:
  ::automap_pro::msg::KeyFrameInfoMsg msg_;
};

class Init_KeyFrameInfoMsg_is_degenerate
{
public:
  explicit Init_KeyFrameInfoMsg_is_degenerate(::automap_pro::msg::KeyFrameInfoMsg & msg)
  : msg_(msg)
  {}
  Init_KeyFrameInfoMsg_map_update_rate is_degenerate(::automap_pro::msg::KeyFrameInfoMsg::_is_degenerate_type arg)
  {
    msg_.is_degenerate = std::move(arg);
    return Init_KeyFrameInfoMsg_map_update_rate(msg_);
  }

private:
  ::automap_pro::msg::KeyFrameInfoMsg msg_;
};

class Init_KeyFrameInfoMsg_esikf_covariance_norm
{
public:
  explicit Init_KeyFrameInfoMsg_esikf_covariance_norm(::automap_pro::msg::KeyFrameInfoMsg & msg)
  : msg_(msg)
  {}
  Init_KeyFrameInfoMsg_is_degenerate esikf_covariance_norm(::automap_pro::msg::KeyFrameInfoMsg::_esikf_covariance_norm_type arg)
  {
    msg_.esikf_covariance_norm = std::move(arg);
    return Init_KeyFrameInfoMsg_is_degenerate(msg_);
  }

private:
  ::automap_pro::msg::KeyFrameInfoMsg msg_;
};

class Init_KeyFrameInfoMsg_timestamp
{
public:
  explicit Init_KeyFrameInfoMsg_timestamp(::automap_pro::msg::KeyFrameInfoMsg & msg)
  : msg_(msg)
  {}
  Init_KeyFrameInfoMsg_esikf_covariance_norm timestamp(::automap_pro::msg::KeyFrameInfoMsg::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_KeyFrameInfoMsg_esikf_covariance_norm(msg_);
  }

private:
  ::automap_pro::msg::KeyFrameInfoMsg msg_;
};

class Init_KeyFrameInfoMsg_keyframe_id
{
public:
  explicit Init_KeyFrameInfoMsg_keyframe_id(::automap_pro::msg::KeyFrameInfoMsg & msg)
  : msg_(msg)
  {}
  Init_KeyFrameInfoMsg_timestamp keyframe_id(::automap_pro::msg::KeyFrameInfoMsg::_keyframe_id_type arg)
  {
    msg_.keyframe_id = std::move(arg);
    return Init_KeyFrameInfoMsg_timestamp(msg_);
  }

private:
  ::automap_pro::msg::KeyFrameInfoMsg msg_;
};

class Init_KeyFrameInfoMsg_header
{
public:
  Init_KeyFrameInfoMsg_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_KeyFrameInfoMsg_keyframe_id header(::automap_pro::msg::KeyFrameInfoMsg::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_KeyFrameInfoMsg_keyframe_id(msg_);
  }

private:
  ::automap_pro::msg::KeyFrameInfoMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::automap_pro::msg::KeyFrameInfoMsg>()
{
  return automap_pro::msg::builder::Init_KeyFrameInfoMsg_header();
}

}  // namespace automap_pro

#endif  // AUTOMAP_PRO__MSG__DETAIL__KEY_FRAME_INFO_MSG__BUILDER_HPP_
