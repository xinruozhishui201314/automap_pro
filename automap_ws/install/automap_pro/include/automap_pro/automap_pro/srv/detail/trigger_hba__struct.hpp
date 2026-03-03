// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from automap_pro:srv/TriggerHBA.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__SRV__DETAIL__TRIGGER_HBA__STRUCT_HPP_
#define AUTOMAP_PRO__SRV__DETAIL__TRIGGER_HBA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__automap_pro__srv__TriggerHBA_Request __attribute__((deprecated))
#else
# define DEPRECATED__automap_pro__srv__TriggerHBA_Request __declspec(deprecated)
#endif

namespace automap_pro
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct TriggerHBA_Request_
{
  using Type = TriggerHBA_Request_<ContainerAllocator>;

  explicit TriggerHBA_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->wait_for_result = false;
    }
  }

  explicit TriggerHBA_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->wait_for_result = false;
    }
  }

  // field types and members
  using _wait_for_result_type =
    bool;
  _wait_for_result_type wait_for_result;

  // setters for named parameter idiom
  Type & set__wait_for_result(
    const bool & _arg)
  {
    this->wait_for_result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    automap_pro::srv::TriggerHBA_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const automap_pro::srv::TriggerHBA_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<automap_pro::srv::TriggerHBA_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<automap_pro::srv::TriggerHBA_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      automap_pro::srv::TriggerHBA_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<automap_pro::srv::TriggerHBA_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      automap_pro::srv::TriggerHBA_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<automap_pro::srv::TriggerHBA_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<automap_pro::srv::TriggerHBA_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<automap_pro::srv::TriggerHBA_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__automap_pro__srv__TriggerHBA_Request
    std::shared_ptr<automap_pro::srv::TriggerHBA_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__automap_pro__srv__TriggerHBA_Request
    std::shared_ptr<automap_pro::srv::TriggerHBA_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TriggerHBA_Request_ & other) const
  {
    if (this->wait_for_result != other.wait_for_result) {
      return false;
    }
    return true;
  }
  bool operator!=(const TriggerHBA_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TriggerHBA_Request_

// alias to use template instance with default allocator
using TriggerHBA_Request =
  automap_pro::srv::TriggerHBA_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace automap_pro


#ifndef _WIN32
# define DEPRECATED__automap_pro__srv__TriggerHBA_Response __attribute__((deprecated))
#else
# define DEPRECATED__automap_pro__srv__TriggerHBA_Response __declspec(deprecated)
#endif

namespace automap_pro
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct TriggerHBA_Response_
{
  using Type = TriggerHBA_Response_<ContainerAllocator>;

  explicit TriggerHBA_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
      this->elapsed_seconds = 0.0;
      this->final_mme = 0.0;
    }
  }

  explicit TriggerHBA_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
      this->elapsed_seconds = 0.0;
      this->final_mme = 0.0;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;
  using _elapsed_seconds_type =
    double;
  _elapsed_seconds_type elapsed_seconds;
  using _final_mme_type =
    double;
  _final_mme_type final_mme;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }
  Type & set__elapsed_seconds(
    const double & _arg)
  {
    this->elapsed_seconds = _arg;
    return *this;
  }
  Type & set__final_mme(
    const double & _arg)
  {
    this->final_mme = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    automap_pro::srv::TriggerHBA_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const automap_pro::srv::TriggerHBA_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<automap_pro::srv::TriggerHBA_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<automap_pro::srv::TriggerHBA_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      automap_pro::srv::TriggerHBA_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<automap_pro::srv::TriggerHBA_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      automap_pro::srv::TriggerHBA_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<automap_pro::srv::TriggerHBA_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<automap_pro::srv::TriggerHBA_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<automap_pro::srv::TriggerHBA_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__automap_pro__srv__TriggerHBA_Response
    std::shared_ptr<automap_pro::srv::TriggerHBA_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__automap_pro__srv__TriggerHBA_Response
    std::shared_ptr<automap_pro::srv::TriggerHBA_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TriggerHBA_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    if (this->elapsed_seconds != other.elapsed_seconds) {
      return false;
    }
    if (this->final_mme != other.final_mme) {
      return false;
    }
    return true;
  }
  bool operator!=(const TriggerHBA_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TriggerHBA_Response_

// alias to use template instance with default allocator
using TriggerHBA_Response =
  automap_pro::srv::TriggerHBA_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace automap_pro

namespace automap_pro
{

namespace srv
{

struct TriggerHBA
{
  using Request = automap_pro::srv::TriggerHBA_Request;
  using Response = automap_pro::srv::TriggerHBA_Response;
};

}  // namespace srv

}  // namespace automap_pro

#endif  // AUTOMAP_PRO__SRV__DETAIL__TRIGGER_HBA__STRUCT_HPP_
