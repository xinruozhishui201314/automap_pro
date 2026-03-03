// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from automap_pro:msg/SubMapEventMsg.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__MSG__DETAIL__SUB_MAP_EVENT_MSG__STRUCT_HPP_
#define AUTOMAP_PRO__MSG__DETAIL__SUB_MAP_EVENT_MSG__STRUCT_HPP_

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
// Member 'anchor_pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__automap_pro__msg__SubMapEventMsg __attribute__((deprecated))
#else
# define DEPRECATED__automap_pro__msg__SubMapEventMsg __declspec(deprecated)
#endif

namespace automap_pro
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SubMapEventMsg_
{
  using Type = SubMapEventMsg_<ContainerAllocator>;

  explicit SubMapEventMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    anchor_pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->submap_id = 0l;
      this->session_id = 0ull;
      this->event_type = "";
      this->keyframe_count = 0l;
      this->spatial_extent_m = 0.0;
      this->has_valid_gps = false;
    }
  }

  explicit SubMapEventMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    event_type(_alloc),
    anchor_pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->submap_id = 0l;
      this->session_id = 0ull;
      this->event_type = "";
      this->keyframe_count = 0l;
      this->spatial_extent_m = 0.0;
      this->has_valid_gps = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _submap_id_type =
    int32_t;
  _submap_id_type submap_id;
  using _session_id_type =
    uint64_t;
  _session_id_type session_id;
  using _event_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _event_type_type event_type;
  using _keyframe_count_type =
    int32_t;
  _keyframe_count_type keyframe_count;
  using _spatial_extent_m_type =
    double;
  _spatial_extent_m_type spatial_extent_m;
  using _anchor_pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _anchor_pose_type anchor_pose;
  using _has_valid_gps_type =
    bool;
  _has_valid_gps_type has_valid_gps;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__submap_id(
    const int32_t & _arg)
  {
    this->submap_id = _arg;
    return *this;
  }
  Type & set__session_id(
    const uint64_t & _arg)
  {
    this->session_id = _arg;
    return *this;
  }
  Type & set__event_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->event_type = _arg;
    return *this;
  }
  Type & set__keyframe_count(
    const int32_t & _arg)
  {
    this->keyframe_count = _arg;
    return *this;
  }
  Type & set__spatial_extent_m(
    const double & _arg)
  {
    this->spatial_extent_m = _arg;
    return *this;
  }
  Type & set__anchor_pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->anchor_pose = _arg;
    return *this;
  }
  Type & set__has_valid_gps(
    const bool & _arg)
  {
    this->has_valid_gps = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    automap_pro::msg::SubMapEventMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const automap_pro::msg::SubMapEventMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<automap_pro::msg::SubMapEventMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<automap_pro::msg::SubMapEventMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      automap_pro::msg::SubMapEventMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<automap_pro::msg::SubMapEventMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      automap_pro::msg::SubMapEventMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<automap_pro::msg::SubMapEventMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<automap_pro::msg::SubMapEventMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<automap_pro::msg::SubMapEventMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__automap_pro__msg__SubMapEventMsg
    std::shared_ptr<automap_pro::msg::SubMapEventMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__automap_pro__msg__SubMapEventMsg
    std::shared_ptr<automap_pro::msg::SubMapEventMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SubMapEventMsg_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->submap_id != other.submap_id) {
      return false;
    }
    if (this->session_id != other.session_id) {
      return false;
    }
    if (this->event_type != other.event_type) {
      return false;
    }
    if (this->keyframe_count != other.keyframe_count) {
      return false;
    }
    if (this->spatial_extent_m != other.spatial_extent_m) {
      return false;
    }
    if (this->anchor_pose != other.anchor_pose) {
      return false;
    }
    if (this->has_valid_gps != other.has_valid_gps) {
      return false;
    }
    return true;
  }
  bool operator!=(const SubMapEventMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SubMapEventMsg_

// alias to use template instance with default allocator
using SubMapEventMsg =
  automap_pro::msg::SubMapEventMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace automap_pro

#endif  // AUTOMAP_PRO__MSG__DETAIL__SUB_MAP_EVENT_MSG__STRUCT_HPP_
