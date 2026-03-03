// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from overlap_transformer_msgs:srv/ComputeDescriptor.idl
// generated code does not contain a copyright notice

#ifndef OVERLAP_TRANSFORMER_MSGS__SRV__DETAIL__COMPUTE_DESCRIPTOR__STRUCT_HPP_
#define OVERLAP_TRANSFORMER_MSGS__SRV__DETAIL__COMPUTE_DESCRIPTOR__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'pointcloud'
#include "sensor_msgs/msg/detail/point_cloud2__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__overlap_transformer_msgs__srv__ComputeDescriptor_Request __attribute__((deprecated))
#else
# define DEPRECATED__overlap_transformer_msgs__srv__ComputeDescriptor_Request __declspec(deprecated)
#endif

namespace overlap_transformer_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ComputeDescriptor_Request_
{
  using Type = ComputeDescriptor_Request_<ContainerAllocator>;

  explicit ComputeDescriptor_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pointcloud(_init)
  {
    (void)_init;
  }

  explicit ComputeDescriptor_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pointcloud(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _pointcloud_type =
    sensor_msgs::msg::PointCloud2_<ContainerAllocator>;
  _pointcloud_type pointcloud;

  // setters for named parameter idiom
  Type & set__pointcloud(
    const sensor_msgs::msg::PointCloud2_<ContainerAllocator> & _arg)
  {
    this->pointcloud = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    overlap_transformer_msgs::srv::ComputeDescriptor_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const overlap_transformer_msgs::srv::ComputeDescriptor_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<overlap_transformer_msgs::srv::ComputeDescriptor_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<overlap_transformer_msgs::srv::ComputeDescriptor_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      overlap_transformer_msgs::srv::ComputeDescriptor_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<overlap_transformer_msgs::srv::ComputeDescriptor_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      overlap_transformer_msgs::srv::ComputeDescriptor_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<overlap_transformer_msgs::srv::ComputeDescriptor_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<overlap_transformer_msgs::srv::ComputeDescriptor_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<overlap_transformer_msgs::srv::ComputeDescriptor_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__overlap_transformer_msgs__srv__ComputeDescriptor_Request
    std::shared_ptr<overlap_transformer_msgs::srv::ComputeDescriptor_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__overlap_transformer_msgs__srv__ComputeDescriptor_Request
    std::shared_ptr<overlap_transformer_msgs::srv::ComputeDescriptor_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ComputeDescriptor_Request_ & other) const
  {
    if (this->pointcloud != other.pointcloud) {
      return false;
    }
    return true;
  }
  bool operator!=(const ComputeDescriptor_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ComputeDescriptor_Request_

// alias to use template instance with default allocator
using ComputeDescriptor_Request =
  overlap_transformer_msgs::srv::ComputeDescriptor_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace overlap_transformer_msgs


// Include directives for member types
// Member 'descriptor'
#include "std_msgs/msg/detail/float32_multi_array__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__overlap_transformer_msgs__srv__ComputeDescriptor_Response __attribute__((deprecated))
#else
# define DEPRECATED__overlap_transformer_msgs__srv__ComputeDescriptor_Response __declspec(deprecated)
#endif

namespace overlap_transformer_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ComputeDescriptor_Response_
{
  using Type = ComputeDescriptor_Response_<ContainerAllocator>;

  explicit ComputeDescriptor_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : descriptor(_init)
  {
    (void)_init;
  }

  explicit ComputeDescriptor_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : descriptor(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _descriptor_type =
    std_msgs::msg::Float32MultiArray_<ContainerAllocator>;
  _descriptor_type descriptor;

  // setters for named parameter idiom
  Type & set__descriptor(
    const std_msgs::msg::Float32MultiArray_<ContainerAllocator> & _arg)
  {
    this->descriptor = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    overlap_transformer_msgs::srv::ComputeDescriptor_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const overlap_transformer_msgs::srv::ComputeDescriptor_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<overlap_transformer_msgs::srv::ComputeDescriptor_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<overlap_transformer_msgs::srv::ComputeDescriptor_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      overlap_transformer_msgs::srv::ComputeDescriptor_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<overlap_transformer_msgs::srv::ComputeDescriptor_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      overlap_transformer_msgs::srv::ComputeDescriptor_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<overlap_transformer_msgs::srv::ComputeDescriptor_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<overlap_transformer_msgs::srv::ComputeDescriptor_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<overlap_transformer_msgs::srv::ComputeDescriptor_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__overlap_transformer_msgs__srv__ComputeDescriptor_Response
    std::shared_ptr<overlap_transformer_msgs::srv::ComputeDescriptor_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__overlap_transformer_msgs__srv__ComputeDescriptor_Response
    std::shared_ptr<overlap_transformer_msgs::srv::ComputeDescriptor_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ComputeDescriptor_Response_ & other) const
  {
    if (this->descriptor != other.descriptor) {
      return false;
    }
    return true;
  }
  bool operator!=(const ComputeDescriptor_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ComputeDescriptor_Response_

// alias to use template instance with default allocator
using ComputeDescriptor_Response =
  overlap_transformer_msgs::srv::ComputeDescriptor_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace overlap_transformer_msgs

namespace overlap_transformer_msgs
{

namespace srv
{

struct ComputeDescriptor
{
  using Request = overlap_transformer_msgs::srv::ComputeDescriptor_Request;
  using Response = overlap_transformer_msgs::srv::ComputeDescriptor_Response;
};

}  // namespace srv

}  // namespace overlap_transformer_msgs

#endif  // OVERLAP_TRANSFORMER_MSGS__SRV__DETAIL__COMPUTE_DESCRIPTOR__STRUCT_HPP_
