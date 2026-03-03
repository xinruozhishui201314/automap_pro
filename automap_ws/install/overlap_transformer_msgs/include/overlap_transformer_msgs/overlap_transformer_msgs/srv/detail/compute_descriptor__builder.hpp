// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from overlap_transformer_msgs:srv/ComputeDescriptor.idl
// generated code does not contain a copyright notice

#ifndef OVERLAP_TRANSFORMER_MSGS__SRV__DETAIL__COMPUTE_DESCRIPTOR__BUILDER_HPP_
#define OVERLAP_TRANSFORMER_MSGS__SRV__DETAIL__COMPUTE_DESCRIPTOR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "overlap_transformer_msgs/srv/detail/compute_descriptor__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace overlap_transformer_msgs
{

namespace srv
{

namespace builder
{

class Init_ComputeDescriptor_Request_pointcloud
{
public:
  Init_ComputeDescriptor_Request_pointcloud()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::overlap_transformer_msgs::srv::ComputeDescriptor_Request pointcloud(::overlap_transformer_msgs::srv::ComputeDescriptor_Request::_pointcloud_type arg)
  {
    msg_.pointcloud = std::move(arg);
    return std::move(msg_);
  }

private:
  ::overlap_transformer_msgs::srv::ComputeDescriptor_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::overlap_transformer_msgs::srv::ComputeDescriptor_Request>()
{
  return overlap_transformer_msgs::srv::builder::Init_ComputeDescriptor_Request_pointcloud();
}

}  // namespace overlap_transformer_msgs


namespace overlap_transformer_msgs
{

namespace srv
{

namespace builder
{

class Init_ComputeDescriptor_Response_descriptor
{
public:
  Init_ComputeDescriptor_Response_descriptor()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::overlap_transformer_msgs::srv::ComputeDescriptor_Response descriptor(::overlap_transformer_msgs::srv::ComputeDescriptor_Response::_descriptor_type arg)
  {
    msg_.descriptor = std::move(arg);
    return std::move(msg_);
  }

private:
  ::overlap_transformer_msgs::srv::ComputeDescriptor_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::overlap_transformer_msgs::srv::ComputeDescriptor_Response>()
{
  return overlap_transformer_msgs::srv::builder::Init_ComputeDescriptor_Response_descriptor();
}

}  // namespace overlap_transformer_msgs

#endif  // OVERLAP_TRANSFORMER_MSGS__SRV__DETAIL__COMPUTE_DESCRIPTOR__BUILDER_HPP_
