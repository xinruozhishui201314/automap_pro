// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from automap_pro:srv/TriggerOptimize.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__SRV__DETAIL__TRIGGER_OPTIMIZE__STRUCT_HPP_
#define AUTOMAP_PRO__SRV__DETAIL__TRIGGER_OPTIMIZE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__automap_pro__srv__TriggerOptimize_Request __attribute__((deprecated))
#else
# define DEPRECATED__automap_pro__srv__TriggerOptimize_Request __declspec(deprecated)
#endif

namespace automap_pro
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct TriggerOptimize_Request_
{
  using Type = TriggerOptimize_Request_<ContainerAllocator>;

  explicit TriggerOptimize_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->full_optimization = false;
      this->max_iterations = 0l;
    }
  }

  explicit TriggerOptimize_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->full_optimization = false;
      this->max_iterations = 0l;
    }
  }

  // field types and members
  using _full_optimization_type =
    bool;
  _full_optimization_type full_optimization;
  using _max_iterations_type =
    int32_t;
  _max_iterations_type max_iterations;

  // setters for named parameter idiom
  Type & set__full_optimization(
    const bool & _arg)
  {
    this->full_optimization = _arg;
    return *this;
  }
  Type & set__max_iterations(
    const int32_t & _arg)
  {
    this->max_iterations = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    automap_pro::srv::TriggerOptimize_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const automap_pro::srv::TriggerOptimize_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<automap_pro::srv::TriggerOptimize_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<automap_pro::srv::TriggerOptimize_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      automap_pro::srv::TriggerOptimize_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<automap_pro::srv::TriggerOptimize_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      automap_pro::srv::TriggerOptimize_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<automap_pro::srv::TriggerOptimize_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<automap_pro::srv::TriggerOptimize_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<automap_pro::srv::TriggerOptimize_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__automap_pro__srv__TriggerOptimize_Request
    std::shared_ptr<automap_pro::srv::TriggerOptimize_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__automap_pro__srv__TriggerOptimize_Request
    std::shared_ptr<automap_pro::srv::TriggerOptimize_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TriggerOptimize_Request_ & other) const
  {
    if (this->full_optimization != other.full_optimization) {
      return false;
    }
    if (this->max_iterations != other.max_iterations) {
      return false;
    }
    return true;
  }
  bool operator!=(const TriggerOptimize_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TriggerOptimize_Request_

// alias to use template instance with default allocator
using TriggerOptimize_Request =
  automap_pro::srv::TriggerOptimize_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace automap_pro


#ifndef _WIN32
# define DEPRECATED__automap_pro__srv__TriggerOptimize_Response __attribute__((deprecated))
#else
# define DEPRECATED__automap_pro__srv__TriggerOptimize_Response __declspec(deprecated)
#endif

namespace automap_pro
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct TriggerOptimize_Response_
{
  using Type = TriggerOptimize_Response_<ContainerAllocator>;

  explicit TriggerOptimize_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->elapsed_seconds = 0.0;
      this->nodes_updated = 0l;
    }
  }

  explicit TriggerOptimize_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->elapsed_seconds = 0.0;
      this->nodes_updated = 0l;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _elapsed_seconds_type =
    double;
  _elapsed_seconds_type elapsed_seconds;
  using _nodes_updated_type =
    int32_t;
  _nodes_updated_type nodes_updated;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__elapsed_seconds(
    const double & _arg)
  {
    this->elapsed_seconds = _arg;
    return *this;
  }
  Type & set__nodes_updated(
    const int32_t & _arg)
  {
    this->nodes_updated = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    automap_pro::srv::TriggerOptimize_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const automap_pro::srv::TriggerOptimize_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<automap_pro::srv::TriggerOptimize_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<automap_pro::srv::TriggerOptimize_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      automap_pro::srv::TriggerOptimize_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<automap_pro::srv::TriggerOptimize_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      automap_pro::srv::TriggerOptimize_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<automap_pro::srv::TriggerOptimize_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<automap_pro::srv::TriggerOptimize_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<automap_pro::srv::TriggerOptimize_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__automap_pro__srv__TriggerOptimize_Response
    std::shared_ptr<automap_pro::srv::TriggerOptimize_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__automap_pro__srv__TriggerOptimize_Response
    std::shared_ptr<automap_pro::srv::TriggerOptimize_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TriggerOptimize_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->elapsed_seconds != other.elapsed_seconds) {
      return false;
    }
    if (this->nodes_updated != other.nodes_updated) {
      return false;
    }
    return true;
  }
  bool operator!=(const TriggerOptimize_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TriggerOptimize_Response_

// alias to use template instance with default allocator
using TriggerOptimize_Response =
  automap_pro::srv::TriggerOptimize_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace automap_pro

namespace automap_pro
{

namespace srv
{

struct TriggerOptimize
{
  using Request = automap_pro::srv::TriggerOptimize_Request;
  using Response = automap_pro::srv::TriggerOptimize_Response;
};

}  // namespace srv

}  // namespace automap_pro

#endif  // AUTOMAP_PRO__SRV__DETAIL__TRIGGER_OPTIMIZE__STRUCT_HPP_
