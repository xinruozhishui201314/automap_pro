// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from automap_pro:srv/GetStatus.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__SRV__DETAIL__GET_STATUS__STRUCT_HPP_
#define AUTOMAP_PRO__SRV__DETAIL__GET_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__automap_pro__srv__GetStatus_Request __attribute__((deprecated))
#else
# define DEPRECATED__automap_pro__srv__GetStatus_Request __declspec(deprecated)
#endif

namespace automap_pro
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetStatus_Request_
{
  using Type = GetStatus_Request_<ContainerAllocator>;

  explicit GetStatus_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit GetStatus_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    automap_pro::srv::GetStatus_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const automap_pro::srv::GetStatus_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<automap_pro::srv::GetStatus_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<automap_pro::srv::GetStatus_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      automap_pro::srv::GetStatus_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<automap_pro::srv::GetStatus_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      automap_pro::srv::GetStatus_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<automap_pro::srv::GetStatus_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<automap_pro::srv::GetStatus_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<automap_pro::srv::GetStatus_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__automap_pro__srv__GetStatus_Request
    std::shared_ptr<automap_pro::srv::GetStatus_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__automap_pro__srv__GetStatus_Request
    std::shared_ptr<automap_pro::srv::GetStatus_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetStatus_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetStatus_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetStatus_Request_

// alias to use template instance with default allocator
using GetStatus_Request =
  automap_pro::srv::GetStatus_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace automap_pro


#ifndef _WIN32
# define DEPRECATED__automap_pro__srv__GetStatus_Response __attribute__((deprecated))
#else
# define DEPRECATED__automap_pro__srv__GetStatus_Response __declspec(deprecated)
#endif

namespace automap_pro
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetStatus_Response_
{
  using Type = GetStatus_Response_<ContainerAllocator>;

  explicit GetStatus_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
      this->gps_align_score = 0.0f;
      this->total_distance_m = 0.0;
    }
  }

  explicit GetStatus_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : state(_alloc)
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
      this->gps_align_score = 0.0f;
      this->total_distance_m = 0.0;
    }
  }

  // field types and members
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
  using _gps_align_score_type =
    float;
  _gps_align_score_type gps_align_score;
  using _total_distance_m_type =
    double;
  _total_distance_m_type total_distance_m;

  // setters for named parameter idiom
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
  Type & set__gps_align_score(
    const float & _arg)
  {
    this->gps_align_score = _arg;
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
    automap_pro::srv::GetStatus_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const automap_pro::srv::GetStatus_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<automap_pro::srv::GetStatus_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<automap_pro::srv::GetStatus_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      automap_pro::srv::GetStatus_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<automap_pro::srv::GetStatus_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      automap_pro::srv::GetStatus_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<automap_pro::srv::GetStatus_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<automap_pro::srv::GetStatus_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<automap_pro::srv::GetStatus_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__automap_pro__srv__GetStatus_Response
    std::shared_ptr<automap_pro::srv::GetStatus_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__automap_pro__srv__GetStatus_Response
    std::shared_ptr<automap_pro::srv::GetStatus_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetStatus_Response_ & other) const
  {
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
    if (this->gps_align_score != other.gps_align_score) {
      return false;
    }
    if (this->total_distance_m != other.total_distance_m) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetStatus_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetStatus_Response_

// alias to use template instance with default allocator
using GetStatus_Response =
  automap_pro::srv::GetStatus_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace automap_pro

namespace automap_pro
{

namespace srv
{

struct GetStatus
{
  using Request = automap_pro::srv::GetStatus_Request;
  using Response = automap_pro::srv::GetStatus_Response;
};

}  // namespace srv

}  // namespace automap_pro

#endif  // AUTOMAP_PRO__SRV__DETAIL__GET_STATUS__STRUCT_HPP_
