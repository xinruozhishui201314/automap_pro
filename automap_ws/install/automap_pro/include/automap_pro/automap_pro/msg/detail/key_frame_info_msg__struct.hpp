// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from automap_pro:msg/KeyFrameInfoMsg.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__MSG__DETAIL__KEY_FRAME_INFO_MSG__STRUCT_HPP_
#define AUTOMAP_PRO__MSG__DETAIL__KEY_FRAME_INFO_MSG__STRUCT_HPP_

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

#ifndef _WIN32
# define DEPRECATED__automap_pro__msg__KeyFrameInfoMsg __attribute__((deprecated))
#else
# define DEPRECATED__automap_pro__msg__KeyFrameInfoMsg __declspec(deprecated)
#endif

namespace automap_pro
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct KeyFrameInfoMsg_
{
  using Type = KeyFrameInfoMsg_<ContainerAllocator>;

  explicit KeyFrameInfoMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->keyframe_id = 0ull;
      this->timestamp = 0.0;
      this->esikf_covariance_norm = 0.0;
      this->is_degenerate = false;
      this->map_update_rate = 0.0f;
      std::fill<typename std::array<double, 3>::iterator, double>(this->gyro_bias.begin(), this->gyro_bias.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->accel_bias.begin(), this->accel_bias.end(), 0.0);
      this->cloud_point_count = 0ul;
      this->cloud_valid = false;
    }
  }

  explicit KeyFrameInfoMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    gyro_bias(_alloc),
    accel_bias(_alloc),
    pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->keyframe_id = 0ull;
      this->timestamp = 0.0;
      this->esikf_covariance_norm = 0.0;
      this->is_degenerate = false;
      this->map_update_rate = 0.0f;
      std::fill<typename std::array<double, 3>::iterator, double>(this->gyro_bias.begin(), this->gyro_bias.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->accel_bias.begin(), this->accel_bias.end(), 0.0);
      this->cloud_point_count = 0ul;
      this->cloud_valid = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _keyframe_id_type =
    uint64_t;
  _keyframe_id_type keyframe_id;
  using _timestamp_type =
    double;
  _timestamp_type timestamp;
  using _esikf_covariance_norm_type =
    double;
  _esikf_covariance_norm_type esikf_covariance_norm;
  using _is_degenerate_type =
    bool;
  _is_degenerate_type is_degenerate;
  using _map_update_rate_type =
    float;
  _map_update_rate_type map_update_rate;
  using _gyro_bias_type =
    std::array<double, 3>;
  _gyro_bias_type gyro_bias;
  using _accel_bias_type =
    std::array<double, 3>;
  _accel_bias_type accel_bias;
  using _cloud_point_count_type =
    uint32_t;
  _cloud_point_count_type cloud_point_count;
  using _cloud_valid_type =
    bool;
  _cloud_valid_type cloud_valid;
  using _pose_type =
    geometry_msgs::msg::PoseWithCovariance_<ContainerAllocator>;
  _pose_type pose;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__keyframe_id(
    const uint64_t & _arg)
  {
    this->keyframe_id = _arg;
    return *this;
  }
  Type & set__timestamp(
    const double & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }
  Type & set__esikf_covariance_norm(
    const double & _arg)
  {
    this->esikf_covariance_norm = _arg;
    return *this;
  }
  Type & set__is_degenerate(
    const bool & _arg)
  {
    this->is_degenerate = _arg;
    return *this;
  }
  Type & set__map_update_rate(
    const float & _arg)
  {
    this->map_update_rate = _arg;
    return *this;
  }
  Type & set__gyro_bias(
    const std::array<double, 3> & _arg)
  {
    this->gyro_bias = _arg;
    return *this;
  }
  Type & set__accel_bias(
    const std::array<double, 3> & _arg)
  {
    this->accel_bias = _arg;
    return *this;
  }
  Type & set__cloud_point_count(
    const uint32_t & _arg)
  {
    this->cloud_point_count = _arg;
    return *this;
  }
  Type & set__cloud_valid(
    const bool & _arg)
  {
    this->cloud_valid = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::PoseWithCovariance_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    automap_pro::msg::KeyFrameInfoMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const automap_pro::msg::KeyFrameInfoMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<automap_pro::msg::KeyFrameInfoMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<automap_pro::msg::KeyFrameInfoMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      automap_pro::msg::KeyFrameInfoMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<automap_pro::msg::KeyFrameInfoMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      automap_pro::msg::KeyFrameInfoMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<automap_pro::msg::KeyFrameInfoMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<automap_pro::msg::KeyFrameInfoMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<automap_pro::msg::KeyFrameInfoMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__automap_pro__msg__KeyFrameInfoMsg
    std::shared_ptr<automap_pro::msg::KeyFrameInfoMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__automap_pro__msg__KeyFrameInfoMsg
    std::shared_ptr<automap_pro::msg::KeyFrameInfoMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const KeyFrameInfoMsg_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->keyframe_id != other.keyframe_id) {
      return false;
    }
    if (this->timestamp != other.timestamp) {
      return false;
    }
    if (this->esikf_covariance_norm != other.esikf_covariance_norm) {
      return false;
    }
    if (this->is_degenerate != other.is_degenerate) {
      return false;
    }
    if (this->map_update_rate != other.map_update_rate) {
      return false;
    }
    if (this->gyro_bias != other.gyro_bias) {
      return false;
    }
    if (this->accel_bias != other.accel_bias) {
      return false;
    }
    if (this->cloud_point_count != other.cloud_point_count) {
      return false;
    }
    if (this->cloud_valid != other.cloud_valid) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const KeyFrameInfoMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct KeyFrameInfoMsg_

// alias to use template instance with default allocator
using KeyFrameInfoMsg =
  automap_pro::msg::KeyFrameInfoMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace automap_pro

#endif  // AUTOMAP_PRO__MSG__DETAIL__KEY_FRAME_INFO_MSG__STRUCT_HPP_
