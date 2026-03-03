// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from automap_pro:msg/KeyFrameMsg.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__MSG__DETAIL__KEY_FRAME_MSG__STRUCT_HPP_
#define AUTOMAP_PRO__MSG__DETAIL__KEY_FRAME_MSG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_with_covariance__struct.hpp"
// Member 'cloud'
#include "sensor_msgs/msg/detail/point_cloud2__struct.hpp"
// Member 'gps'
#include "automap_pro/msg/detail/gps_measurement_msg__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__automap_pro__msg__KeyFrameMsg __attribute__((deprecated))
#else
# define DEPRECATED__automap_pro__msg__KeyFrameMsg __declspec(deprecated)
#endif

namespace automap_pro
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct KeyFrameMsg_
{
  using Type = KeyFrameMsg_<ContainerAllocator>;

  explicit KeyFrameMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    pose(_init),
    cloud(_init),
    gps(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0ull;
      this->session_id = 0ull;
      this->submap_id = 0l;
      this->has_gps = false;
    }
  }

  explicit KeyFrameMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    pose(_alloc, _init),
    cloud(_alloc, _init),
    gps(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0ull;
      this->session_id = 0ull;
      this->submap_id = 0l;
      this->has_gps = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _id_type =
    uint64_t;
  _id_type id;
  using _session_id_type =
    uint64_t;
  _session_id_type session_id;
  using _submap_id_type =
    int32_t;
  _submap_id_type submap_id;
  using _pose_type =
    geometry_msgs::msg::PoseWithCovariance_<ContainerAllocator>;
  _pose_type pose;
  using _cloud_type =
    sensor_msgs::msg::PointCloud2_<ContainerAllocator>;
  _cloud_type cloud;
  using _has_gps_type =
    bool;
  _has_gps_type has_gps;
  using _gps_type =
    automap_pro::msg::GPSMeasurementMsg_<ContainerAllocator>;
  _gps_type gps;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__id(
    const uint64_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__session_id(
    const uint64_t & _arg)
  {
    this->session_id = _arg;
    return *this;
  }
  Type & set__submap_id(
    const int32_t & _arg)
  {
    this->submap_id = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::PoseWithCovariance_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__cloud(
    const sensor_msgs::msg::PointCloud2_<ContainerAllocator> & _arg)
  {
    this->cloud = _arg;
    return *this;
  }
  Type & set__has_gps(
    const bool & _arg)
  {
    this->has_gps = _arg;
    return *this;
  }
  Type & set__gps(
    const automap_pro::msg::GPSMeasurementMsg_<ContainerAllocator> & _arg)
  {
    this->gps = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    automap_pro::msg::KeyFrameMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const automap_pro::msg::KeyFrameMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<automap_pro::msg::KeyFrameMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<automap_pro::msg::KeyFrameMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      automap_pro::msg::KeyFrameMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<automap_pro::msg::KeyFrameMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      automap_pro::msg::KeyFrameMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<automap_pro::msg::KeyFrameMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<automap_pro::msg::KeyFrameMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<automap_pro::msg::KeyFrameMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__automap_pro__msg__KeyFrameMsg
    std::shared_ptr<automap_pro::msg::KeyFrameMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__automap_pro__msg__KeyFrameMsg
    std::shared_ptr<automap_pro::msg::KeyFrameMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const KeyFrameMsg_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    if (this->session_id != other.session_id) {
      return false;
    }
    if (this->submap_id != other.submap_id) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    if (this->cloud != other.cloud) {
      return false;
    }
    if (this->has_gps != other.has_gps) {
      return false;
    }
    if (this->gps != other.gps) {
      return false;
    }
    return true;
  }
  bool operator!=(const KeyFrameMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct KeyFrameMsg_

// alias to use template instance with default allocator
using KeyFrameMsg =
  automap_pro::msg::KeyFrameMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace automap_pro

#endif  // AUTOMAP_PRO__MSG__DETAIL__KEY_FRAME_MSG__STRUCT_HPP_
