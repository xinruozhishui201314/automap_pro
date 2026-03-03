// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from automap_pro:srv/SaveMap.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__SRV__DETAIL__SAVE_MAP__STRUCT_HPP_
#define AUTOMAP_PRO__SRV__DETAIL__SAVE_MAP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__automap_pro__srv__SaveMap_Request __attribute__((deprecated))
#else
# define DEPRECATED__automap_pro__srv__SaveMap_Request __declspec(deprecated)
#endif

namespace automap_pro
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SaveMap_Request_
{
  using Type = SaveMap_Request_<ContainerAllocator>;

  explicit SaveMap_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->output_dir = "";
      this->save_pcd = false;
      this->save_ply = false;
      this->save_las = false;
      this->save_trajectory_tum = false;
      this->save_trajectory_kitti = false;
    }
  }

  explicit SaveMap_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : output_dir(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->output_dir = "";
      this->save_pcd = false;
      this->save_ply = false;
      this->save_las = false;
      this->save_trajectory_tum = false;
      this->save_trajectory_kitti = false;
    }
  }

  // field types and members
  using _output_dir_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _output_dir_type output_dir;
  using _save_pcd_type =
    bool;
  _save_pcd_type save_pcd;
  using _save_ply_type =
    bool;
  _save_ply_type save_ply;
  using _save_las_type =
    bool;
  _save_las_type save_las;
  using _save_trajectory_tum_type =
    bool;
  _save_trajectory_tum_type save_trajectory_tum;
  using _save_trajectory_kitti_type =
    bool;
  _save_trajectory_kitti_type save_trajectory_kitti;

  // setters for named parameter idiom
  Type & set__output_dir(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->output_dir = _arg;
    return *this;
  }
  Type & set__save_pcd(
    const bool & _arg)
  {
    this->save_pcd = _arg;
    return *this;
  }
  Type & set__save_ply(
    const bool & _arg)
  {
    this->save_ply = _arg;
    return *this;
  }
  Type & set__save_las(
    const bool & _arg)
  {
    this->save_las = _arg;
    return *this;
  }
  Type & set__save_trajectory_tum(
    const bool & _arg)
  {
    this->save_trajectory_tum = _arg;
    return *this;
  }
  Type & set__save_trajectory_kitti(
    const bool & _arg)
  {
    this->save_trajectory_kitti = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    automap_pro::srv::SaveMap_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const automap_pro::srv::SaveMap_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<automap_pro::srv::SaveMap_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<automap_pro::srv::SaveMap_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      automap_pro::srv::SaveMap_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<automap_pro::srv::SaveMap_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      automap_pro::srv::SaveMap_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<automap_pro::srv::SaveMap_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<automap_pro::srv::SaveMap_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<automap_pro::srv::SaveMap_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__automap_pro__srv__SaveMap_Request
    std::shared_ptr<automap_pro::srv::SaveMap_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__automap_pro__srv__SaveMap_Request
    std::shared_ptr<automap_pro::srv::SaveMap_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SaveMap_Request_ & other) const
  {
    if (this->output_dir != other.output_dir) {
      return false;
    }
    if (this->save_pcd != other.save_pcd) {
      return false;
    }
    if (this->save_ply != other.save_ply) {
      return false;
    }
    if (this->save_las != other.save_las) {
      return false;
    }
    if (this->save_trajectory_tum != other.save_trajectory_tum) {
      return false;
    }
    if (this->save_trajectory_kitti != other.save_trajectory_kitti) {
      return false;
    }
    return true;
  }
  bool operator!=(const SaveMap_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SaveMap_Request_

// alias to use template instance with default allocator
using SaveMap_Request =
  automap_pro::srv::SaveMap_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace automap_pro


#ifndef _WIN32
# define DEPRECATED__automap_pro__srv__SaveMap_Response __attribute__((deprecated))
#else
# define DEPRECATED__automap_pro__srv__SaveMap_Response __declspec(deprecated)
#endif

namespace automap_pro
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SaveMap_Response_
{
  using Type = SaveMap_Response_<ContainerAllocator>;

  explicit SaveMap_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->output_path = "";
      this->message = "";
    }
  }

  explicit SaveMap_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : output_path(_alloc),
    message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->output_path = "";
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _output_path_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _output_path_type output_path;
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
  Type & set__output_path(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->output_path = _arg;
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
    automap_pro::srv::SaveMap_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const automap_pro::srv::SaveMap_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<automap_pro::srv::SaveMap_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<automap_pro::srv::SaveMap_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      automap_pro::srv::SaveMap_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<automap_pro::srv::SaveMap_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      automap_pro::srv::SaveMap_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<automap_pro::srv::SaveMap_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<automap_pro::srv::SaveMap_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<automap_pro::srv::SaveMap_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__automap_pro__srv__SaveMap_Response
    std::shared_ptr<automap_pro::srv::SaveMap_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__automap_pro__srv__SaveMap_Response
    std::shared_ptr<automap_pro::srv::SaveMap_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SaveMap_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->output_path != other.output_path) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const SaveMap_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SaveMap_Response_

// alias to use template instance with default allocator
using SaveMap_Response =
  automap_pro::srv::SaveMap_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace automap_pro

namespace automap_pro
{

namespace srv
{

struct SaveMap
{
  using Request = automap_pro::srv::SaveMap_Request;
  using Response = automap_pro::srv::SaveMap_Response;
};

}  // namespace srv

}  // namespace automap_pro

#endif  // AUTOMAP_PRO__SRV__DETAIL__SAVE_MAP__STRUCT_HPP_
