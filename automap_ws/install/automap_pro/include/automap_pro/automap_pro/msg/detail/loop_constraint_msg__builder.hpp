// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from automap_pro:msg/LoopConstraintMsg.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__MSG__DETAIL__LOOP_CONSTRAINT_MSG__BUILDER_HPP_
#define AUTOMAP_PRO__MSG__DETAIL__LOOP_CONSTRAINT_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "automap_pro/msg/detail/loop_constraint_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace automap_pro
{

namespace msg
{

namespace builder
{

class Init_LoopConstraintMsg_status
{
public:
  explicit Init_LoopConstraintMsg_status(::automap_pro::msg::LoopConstraintMsg & msg)
  : msg_(msg)
  {}
  ::automap_pro::msg::LoopConstraintMsg status(::automap_pro::msg::LoopConstraintMsg::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::automap_pro::msg::LoopConstraintMsg msg_;
};

class Init_LoopConstraintMsg_delta_pose
{
public:
  explicit Init_LoopConstraintMsg_delta_pose(::automap_pro::msg::LoopConstraintMsg & msg)
  : msg_(msg)
  {}
  Init_LoopConstraintMsg_status delta_pose(::automap_pro::msg::LoopConstraintMsg::_delta_pose_type arg)
  {
    msg_.delta_pose = std::move(arg);
    return Init_LoopConstraintMsg_status(msg_);
  }

private:
  ::automap_pro::msg::LoopConstraintMsg msg_;
};

class Init_LoopConstraintMsg_information_matrix
{
public:
  explicit Init_LoopConstraintMsg_information_matrix(::automap_pro::msg::LoopConstraintMsg & msg)
  : msg_(msg)
  {}
  Init_LoopConstraintMsg_delta_pose information_matrix(::automap_pro::msg::LoopConstraintMsg::_information_matrix_type arg)
  {
    msg_.information_matrix = std::move(arg);
    return Init_LoopConstraintMsg_delta_pose(msg_);
  }

private:
  ::automap_pro::msg::LoopConstraintMsg msg_;
};

class Init_LoopConstraintMsg_is_inter_session
{
public:
  explicit Init_LoopConstraintMsg_is_inter_session(::automap_pro::msg::LoopConstraintMsg & msg)
  : msg_(msg)
  {}
  Init_LoopConstraintMsg_information_matrix is_inter_session(::automap_pro::msg::LoopConstraintMsg::_is_inter_session_type arg)
  {
    msg_.is_inter_session = std::move(arg);
    return Init_LoopConstraintMsg_information_matrix(msg_);
  }

private:
  ::automap_pro::msg::LoopConstraintMsg msg_;
};

class Init_LoopConstraintMsg_rmse
{
public:
  explicit Init_LoopConstraintMsg_rmse(::automap_pro::msg::LoopConstraintMsg & msg)
  : msg_(msg)
  {}
  Init_LoopConstraintMsg_is_inter_session rmse(::automap_pro::msg::LoopConstraintMsg::_rmse_type arg)
  {
    msg_.rmse = std::move(arg);
    return Init_LoopConstraintMsg_is_inter_session(msg_);
  }

private:
  ::automap_pro::msg::LoopConstraintMsg msg_;
};

class Init_LoopConstraintMsg_inlier_ratio
{
public:
  explicit Init_LoopConstraintMsg_inlier_ratio(::automap_pro::msg::LoopConstraintMsg & msg)
  : msg_(msg)
  {}
  Init_LoopConstraintMsg_rmse inlier_ratio(::automap_pro::msg::LoopConstraintMsg::_inlier_ratio_type arg)
  {
    msg_.inlier_ratio = std::move(arg);
    return Init_LoopConstraintMsg_rmse(msg_);
  }

private:
  ::automap_pro::msg::LoopConstraintMsg msg_;
};

class Init_LoopConstraintMsg_overlap_score
{
public:
  explicit Init_LoopConstraintMsg_overlap_score(::automap_pro::msg::LoopConstraintMsg & msg)
  : msg_(msg)
  {}
  Init_LoopConstraintMsg_inlier_ratio overlap_score(::automap_pro::msg::LoopConstraintMsg::_overlap_score_type arg)
  {
    msg_.overlap_score = std::move(arg);
    return Init_LoopConstraintMsg_inlier_ratio(msg_);
  }

private:
  ::automap_pro::msg::LoopConstraintMsg msg_;
};

class Init_LoopConstraintMsg_session_j
{
public:
  explicit Init_LoopConstraintMsg_session_j(::automap_pro::msg::LoopConstraintMsg & msg)
  : msg_(msg)
  {}
  Init_LoopConstraintMsg_overlap_score session_j(::automap_pro::msg::LoopConstraintMsg::_session_j_type arg)
  {
    msg_.session_j = std::move(arg);
    return Init_LoopConstraintMsg_overlap_score(msg_);
  }

private:
  ::automap_pro::msg::LoopConstraintMsg msg_;
};

class Init_LoopConstraintMsg_session_i
{
public:
  explicit Init_LoopConstraintMsg_session_i(::automap_pro::msg::LoopConstraintMsg & msg)
  : msg_(msg)
  {}
  Init_LoopConstraintMsg_session_j session_i(::automap_pro::msg::LoopConstraintMsg::_session_i_type arg)
  {
    msg_.session_i = std::move(arg);
    return Init_LoopConstraintMsg_session_j(msg_);
  }

private:
  ::automap_pro::msg::LoopConstraintMsg msg_;
};

class Init_LoopConstraintMsg_submap_j
{
public:
  explicit Init_LoopConstraintMsg_submap_j(::automap_pro::msg::LoopConstraintMsg & msg)
  : msg_(msg)
  {}
  Init_LoopConstraintMsg_session_i submap_j(::automap_pro::msg::LoopConstraintMsg::_submap_j_type arg)
  {
    msg_.submap_j = std::move(arg);
    return Init_LoopConstraintMsg_session_i(msg_);
  }

private:
  ::automap_pro::msg::LoopConstraintMsg msg_;
};

class Init_LoopConstraintMsg_submap_i
{
public:
  explicit Init_LoopConstraintMsg_submap_i(::automap_pro::msg::LoopConstraintMsg & msg)
  : msg_(msg)
  {}
  Init_LoopConstraintMsg_submap_j submap_i(::automap_pro::msg::LoopConstraintMsg::_submap_i_type arg)
  {
    msg_.submap_i = std::move(arg);
    return Init_LoopConstraintMsg_submap_j(msg_);
  }

private:
  ::automap_pro::msg::LoopConstraintMsg msg_;
};

class Init_LoopConstraintMsg_header
{
public:
  Init_LoopConstraintMsg_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LoopConstraintMsg_submap_i header(::automap_pro::msg::LoopConstraintMsg::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_LoopConstraintMsg_submap_i(msg_);
  }

private:
  ::automap_pro::msg::LoopConstraintMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::automap_pro::msg::LoopConstraintMsg>()
{
  return automap_pro::msg::builder::Init_LoopConstraintMsg_header();
}

}  // namespace automap_pro

#endif  // AUTOMAP_PRO__MSG__DETAIL__LOOP_CONSTRAINT_MSG__BUILDER_HPP_
