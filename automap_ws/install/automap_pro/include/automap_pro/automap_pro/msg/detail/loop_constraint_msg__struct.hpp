// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from automap_pro:msg/LoopConstraintMsg.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__MSG__DETAIL__LOOP_CONSTRAINT_MSG__STRUCT_HPP_
#define AUTOMAP_PRO__MSG__DETAIL__LOOP_CONSTRAINT_MSG__STRUCT_HPP_

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
// Member 'delta_pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__automap_pro__msg__LoopConstraintMsg __attribute__((deprecated))
#else
# define DEPRECATED__automap_pro__msg__LoopConstraintMsg __declspec(deprecated)
#endif

namespace automap_pro
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LoopConstraintMsg_
{
  using Type = LoopConstraintMsg_<ContainerAllocator>;

  explicit LoopConstraintMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    delta_pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->submap_i = 0l;
      this->submap_j = 0l;
      this->session_i = 0ull;
      this->session_j = 0ull;
      this->overlap_score = 0.0f;
      this->inlier_ratio = 0.0f;
      this->rmse = 0.0f;
      this->is_inter_session = false;
      std::fill<typename std::array<double, 36>::iterator, double>(this->information_matrix.begin(), this->information_matrix.end(), 0.0);
      this->status = "";
    }
  }

  explicit LoopConstraintMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    information_matrix(_alloc),
    delta_pose(_alloc, _init),
    status(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->submap_i = 0l;
      this->submap_j = 0l;
      this->session_i = 0ull;
      this->session_j = 0ull;
      this->overlap_score = 0.0f;
      this->inlier_ratio = 0.0f;
      this->rmse = 0.0f;
      this->is_inter_session = false;
      std::fill<typename std::array<double, 36>::iterator, double>(this->information_matrix.begin(), this->information_matrix.end(), 0.0);
      this->status = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _submap_i_type =
    int32_t;
  _submap_i_type submap_i;
  using _submap_j_type =
    int32_t;
  _submap_j_type submap_j;
  using _session_i_type =
    uint64_t;
  _session_i_type session_i;
  using _session_j_type =
    uint64_t;
  _session_j_type session_j;
  using _overlap_score_type =
    float;
  _overlap_score_type overlap_score;
  using _inlier_ratio_type =
    float;
  _inlier_ratio_type inlier_ratio;
  using _rmse_type =
    float;
  _rmse_type rmse;
  using _is_inter_session_type =
    bool;
  _is_inter_session_type is_inter_session;
  using _information_matrix_type =
    std::array<double, 36>;
  _information_matrix_type information_matrix;
  using _delta_pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _delta_pose_type delta_pose;
  using _status_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _status_type status;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__submap_i(
    const int32_t & _arg)
  {
    this->submap_i = _arg;
    return *this;
  }
  Type & set__submap_j(
    const int32_t & _arg)
  {
    this->submap_j = _arg;
    return *this;
  }
  Type & set__session_i(
    const uint64_t & _arg)
  {
    this->session_i = _arg;
    return *this;
  }
  Type & set__session_j(
    const uint64_t & _arg)
  {
    this->session_j = _arg;
    return *this;
  }
  Type & set__overlap_score(
    const float & _arg)
  {
    this->overlap_score = _arg;
    return *this;
  }
  Type & set__inlier_ratio(
    const float & _arg)
  {
    this->inlier_ratio = _arg;
    return *this;
  }
  Type & set__rmse(
    const float & _arg)
  {
    this->rmse = _arg;
    return *this;
  }
  Type & set__is_inter_session(
    const bool & _arg)
  {
    this->is_inter_session = _arg;
    return *this;
  }
  Type & set__information_matrix(
    const std::array<double, 36> & _arg)
  {
    this->information_matrix = _arg;
    return *this;
  }
  Type & set__delta_pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->delta_pose = _arg;
    return *this;
  }
  Type & set__status(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->status = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    automap_pro::msg::LoopConstraintMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const automap_pro::msg::LoopConstraintMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<automap_pro::msg::LoopConstraintMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<automap_pro::msg::LoopConstraintMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      automap_pro::msg::LoopConstraintMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<automap_pro::msg::LoopConstraintMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      automap_pro::msg::LoopConstraintMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<automap_pro::msg::LoopConstraintMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<automap_pro::msg::LoopConstraintMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<automap_pro::msg::LoopConstraintMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__automap_pro__msg__LoopConstraintMsg
    std::shared_ptr<automap_pro::msg::LoopConstraintMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__automap_pro__msg__LoopConstraintMsg
    std::shared_ptr<automap_pro::msg::LoopConstraintMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LoopConstraintMsg_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->submap_i != other.submap_i) {
      return false;
    }
    if (this->submap_j != other.submap_j) {
      return false;
    }
    if (this->session_i != other.session_i) {
      return false;
    }
    if (this->session_j != other.session_j) {
      return false;
    }
    if (this->overlap_score != other.overlap_score) {
      return false;
    }
    if (this->inlier_ratio != other.inlier_ratio) {
      return false;
    }
    if (this->rmse != other.rmse) {
      return false;
    }
    if (this->is_inter_session != other.is_inter_session) {
      return false;
    }
    if (this->information_matrix != other.information_matrix) {
      return false;
    }
    if (this->delta_pose != other.delta_pose) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const LoopConstraintMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LoopConstraintMsg_

// alias to use template instance with default allocator
using LoopConstraintMsg =
  automap_pro::msg::LoopConstraintMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace automap_pro

#endif  // AUTOMAP_PRO__MSG__DETAIL__LOOP_CONSTRAINT_MSG__STRUCT_HPP_
