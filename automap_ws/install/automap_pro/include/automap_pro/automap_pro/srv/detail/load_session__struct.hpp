// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from automap_pro:srv/LoadSession.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__SRV__DETAIL__LOAD_SESSION__STRUCT_HPP_
#define AUTOMAP_PRO__SRV__DETAIL__LOAD_SESSION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__automap_pro__srv__LoadSession_Request __attribute__((deprecated))
#else
# define DEPRECATED__automap_pro__srv__LoadSession_Request __declspec(deprecated)
#endif

namespace automap_pro
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct LoadSession_Request_
{
  using Type = LoadSession_Request_<ContainerAllocator>;

  explicit LoadSession_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->session_dir = "";
      this->session_id = 0ull;
    }
  }

  explicit LoadSession_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : session_dir(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->session_dir = "";
      this->session_id = 0ull;
    }
  }

  // field types and members
  using _session_dir_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _session_dir_type session_dir;
  using _session_id_type =
    uint64_t;
  _session_id_type session_id;

  // setters for named parameter idiom
  Type & set__session_dir(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->session_dir = _arg;
    return *this;
  }
  Type & set__session_id(
    const uint64_t & _arg)
  {
    this->session_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    automap_pro::srv::LoadSession_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const automap_pro::srv::LoadSession_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<automap_pro::srv::LoadSession_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<automap_pro::srv::LoadSession_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      automap_pro::srv::LoadSession_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<automap_pro::srv::LoadSession_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      automap_pro::srv::LoadSession_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<automap_pro::srv::LoadSession_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<automap_pro::srv::LoadSession_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<automap_pro::srv::LoadSession_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__automap_pro__srv__LoadSession_Request
    std::shared_ptr<automap_pro::srv::LoadSession_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__automap_pro__srv__LoadSession_Request
    std::shared_ptr<automap_pro::srv::LoadSession_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LoadSession_Request_ & other) const
  {
    if (this->session_dir != other.session_dir) {
      return false;
    }
    if (this->session_id != other.session_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const LoadSession_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LoadSession_Request_

// alias to use template instance with default allocator
using LoadSession_Request =
  automap_pro::srv::LoadSession_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace automap_pro


#ifndef _WIN32
# define DEPRECATED__automap_pro__srv__LoadSession_Response __attribute__((deprecated))
#else
# define DEPRECATED__automap_pro__srv__LoadSession_Response __declspec(deprecated)
#endif

namespace automap_pro
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct LoadSession_Response_
{
  using Type = LoadSession_Response_<ContainerAllocator>;

  explicit LoadSession_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->submaps_loaded = 0ul;
      this->descriptors_loaded = 0ul;
      this->message = "";
    }
  }

  explicit LoadSession_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->submaps_loaded = 0ul;
      this->descriptors_loaded = 0ul;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _submaps_loaded_type =
    uint32_t;
  _submaps_loaded_type submaps_loaded;
  using _descriptors_loaded_type =
    uint32_t;
  _descriptors_loaded_type descriptors_loaded;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__submaps_loaded(
    const uint32_t & _arg)
  {
    this->submaps_loaded = _arg;
    return *this;
  }
  Type & set__descriptors_loaded(
    const uint32_t & _arg)
  {
    this->descriptors_loaded = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    automap_pro::srv::LoadSession_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const automap_pro::srv::LoadSession_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<automap_pro::srv::LoadSession_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<automap_pro::srv::LoadSession_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      automap_pro::srv::LoadSession_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<automap_pro::srv::LoadSession_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      automap_pro::srv::LoadSession_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<automap_pro::srv::LoadSession_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<automap_pro::srv::LoadSession_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<automap_pro::srv::LoadSession_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__automap_pro__srv__LoadSession_Response
    std::shared_ptr<automap_pro::srv::LoadSession_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__automap_pro__srv__LoadSession_Response
    std::shared_ptr<automap_pro::srv::LoadSession_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LoadSession_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->submaps_loaded != other.submaps_loaded) {
      return false;
    }
    if (this->descriptors_loaded != other.descriptors_loaded) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const LoadSession_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LoadSession_Response_

// alias to use template instance with default allocator
using LoadSession_Response =
  automap_pro::srv::LoadSession_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace automap_pro

namespace automap_pro
{

namespace srv
{

struct LoadSession
{
  using Request = automap_pro::srv::LoadSession_Request;
  using Response = automap_pro::srv::LoadSession_Response;
};

}  // namespace srv

}  // namespace automap_pro

#endif  // AUTOMAP_PRO__SRV__DETAIL__LOAD_SESSION__STRUCT_HPP_
