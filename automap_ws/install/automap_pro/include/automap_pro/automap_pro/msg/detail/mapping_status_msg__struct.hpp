// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from automap_pro:msg/MappingStatusMsg.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__MSG__DETAIL__MAPPING_STATUS_MSG__STRUCT_HPP_
#define AUTOMAP_PRO__MSG__DETAIL__MAPPING_STATUS_MSG__STRUCT_HPP_

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

#ifndef _WIN32
# define DEPRECATED__automap_pro__msg__MappingStatusMsg __attribute__((deprecated))
#else
# define DEPRECATED__automap_pro__msg__MappingStatusMsg __declspec(deprecated)
#endif

namespace automap_pro
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MappingStatusMsg_
{
  using Type = MappingStatusMsg_<ContainerAllocator>;

  explicit MappingStatusMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = "";
      this->session_id = 0ull;
      this->keyframe_count = 0ul;
      this->submap_count = 0ul;
      this->loop_count = 0ul;
      this->gps_aligned = false;
      this->gps_alignment_score = 0.0f;
      this->map_entropy = 0.0;
      this->total_distance_m = 0.0;
    }
  }

  explicit MappingStatusMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    state(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = "";
      this->session_id = 0ull;
      this->keyframe_count = 0ul;
      this->submap_count = 0ul;
      this->loop_count = 0ul;
      this->gps_aligned = false;
      this->gps_alignment_score = 0.0f;
      this->map_entropy = 0.0;
      this->total_distance_m = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _state_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _state_type state;
  using _session_id_type =
    uint64_t;
  _session_id_type session_id;
  using _keyframe_count_type =
    uint32_t;
  _keyframe_count_type keyframe_count;
  using _submap_count_type =
    uint32_t;
  _submap_count_type submap_count;
  using _loop_count_type =
    uint32_t;
  _loop_count_type loop_count;
  using _gps_aligned_type =
    bool;
  _gps_aligned_type gps_aligned;
  using _gps_alignment_score_type =
    float;
  _gps_alignment_score_type gps_alignment_score;
  using _map_entropy_type =
    double;
  _map_entropy_type map_entropy;
  using _total_distance_m_type =
    double;
  _total_distance_m_type total_distance_m;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__state(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->state = _arg;
    return *this;
  }
  Type & set__session_id(
    const uint64_t & _arg)
  {
    this->session_id = _arg;
    return *this;
  }
  Type & set__keyframe_count(
    const uint32_t & _arg)
  {
    this->keyframe_count = _arg;
    return *this;
  }
  Type & set__submap_count(
    const uint32_t & _arg)
  {
    this->submap_count = _arg;
    return *this;
  }
  Type & set__loop_count(
    const uint32_t & _arg)
  {
    this->loop_count = _arg;
    return *this;
  }
  Type & set__gps_aligned(
    const bool & _arg)
  {
    this->gps_aligned = _arg;
    return *this;
  }
  Type & set__gps_alignment_score(
    const float & _arg)
  {
    this->gps_alignment_score = _arg;
    return *this;
  }
  Type & set__map_entropy(
    const double & _arg)
  {
    this->map_entropy = _arg;
    return *this;
  }
  Type & set__total_distance_m(
    const double & _arg)
  {
    this->total_distance_m = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    automap_pro::msg::MappingStatusMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const automap_pro::msg::MappingStatusMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<automap_pro::msg::MappingStatusMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<automap_pro::msg::MappingStatusMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      automap_pro::msg::MappingStatusMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<automap_pro::msg::MappingStatusMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      automap_pro::msg::MappingStatusMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<automap_pro::msg::MappingStatusMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<automap_pro::msg::MappingStatusMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<automap_pro::msg::MappingStatusMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__automap_pro__msg__MappingStatusMsg
    std::shared_ptr<automap_pro::msg::MappingStatusMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__automap_pro__msg__MappingStatusMsg
    std::shared_ptr<automap_pro::msg::MappingStatusMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MappingStatusMsg_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    if (this->session_id != other.session_id) {
      return false;
    }
    if (this->keyframe_count != other.keyframe_count) {
      return false;
    }
    if (this->submap_count != other.submap_count) {
      return false;
    }
    if (this->loop_count != other.loop_count) {
      return false;
    }
    if (this->gps_aligned != other.gps_aligned) {
      return false;
    }
    if (this->gps_alignment_score != other.gps_alignment_score) {
      return false;
    }
    if (this->map_entropy != other.map_entropy) {
      return false;
    }
    if (this->total_distance_m != other.total_distance_m) {
      return false;
    }
    return true;
  }
  bool operator!=(const MappingStatusMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MappingStatusMsg_

// alias to use template instance with default allocator
using MappingStatusMsg =
  automap_pro::msg::MappingStatusMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace automap_pro

#endif  // AUTOMAP_PRO__MSG__DETAIL__MAPPING_STATUS_MSG__STRUCT_HPP_
