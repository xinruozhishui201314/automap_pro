// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from automap_pro:srv/TriggerGpsAlign.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__SRV__DETAIL__TRIGGER_GPS_ALIGN__STRUCT_HPP_
#define AUTOMAP_PRO__SRV__DETAIL__TRIGGER_GPS_ALIGN__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__automap_pro__srv__TriggerGpsAlign_Request __attribute__((deprecated))
#else
# define DEPRECATED__automap_pro__srv__TriggerGpsAlign_Request __declspec(deprecated)
#endif

namespace automap_pro
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct TriggerGpsAlign_Request_
{
  using Type = TriggerGpsAlign_Request_<ContainerAllocator>;

  explicit TriggerGpsAlign_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->force = false;
    }
  }

  explicit TriggerGpsAlign_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->force = false;
    }
  }

  // field types and members
  using _force_type =
    bool;
  _force_type force;

  // setters for named parameter idiom
  Type & set__force(
    const bool & _arg)
  {
    this->force = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    automap_pro::srv::TriggerGpsAlign_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const automap_pro::srv::TriggerGpsAlign_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<automap_pro::srv::TriggerGpsAlign_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<automap_pro::srv::TriggerGpsAlign_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      automap_pro::srv::TriggerGpsAlign_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<automap_pro::srv::TriggerGpsAlign_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      automap_pro::srv::TriggerGpsAlign_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<automap_pro::srv::TriggerGpsAlign_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<automap_pro::srv::TriggerGpsAlign_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<automap_pro::srv::TriggerGpsAlign_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__automap_pro__srv__TriggerGpsAlign_Request
    std::shared_ptr<automap_pro::srv::TriggerGpsAlign_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__automap_pro__srv__TriggerGpsAlign_Request
    std::shared_ptr<automap_pro::srv::TriggerGpsAlign_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TriggerGpsAlign_Request_ & other) const
  {
    if (this->force != other.force) {
      return false;
    }
    return true;
  }
  bool operator!=(const TriggerGpsAlign_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TriggerGpsAlign_Request_

// alias to use template instance with default allocator
using TriggerGpsAlign_Request =
  automap_pro::srv::TriggerGpsAlign_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace automap_pro


#ifndef _WIN32
# define DEPRECATED__automap_pro__srv__TriggerGpsAlign_Response __attribute__((deprecated))
#else
# define DEPRECATED__automap_pro__srv__TriggerGpsAlign_Response __declspec(deprecated)
#endif

namespace automap_pro
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct TriggerGpsAlign_Response_
{
  using Type = TriggerGpsAlign_Response_<ContainerAllocator>;

  explicit TriggerGpsAlign_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->alignment_rmse_m = 0.0;
      std::fill<typename std::array<double, 9>::iterator, double>(this->r_gps_lidar.begin(), this->r_gps_lidar.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->t_gps_lidar.begin(), this->t_gps_lidar.end(), 0.0);
      this->message = "";
    }
  }

  explicit TriggerGpsAlign_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : r_gps_lidar(_alloc),
    t_gps_lidar(_alloc),
    message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->alignment_rmse_m = 0.0;
      std::fill<typename std::array<double, 9>::iterator, double>(this->r_gps_lidar.begin(), this->r_gps_lidar.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->t_gps_lidar.begin(), this->t_gps_lidar.end(), 0.0);
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _alignment_rmse_m_type =
    double;
  _alignment_rmse_m_type alignment_rmse_m;
  using _r_gps_lidar_type =
    std::array<double, 9>;
  _r_gps_lidar_type r_gps_lidar;
  using _t_gps_lidar_type =
    std::array<double, 3>;
  _t_gps_lidar_type t_gps_lidar;
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
  Type & set__alignment_rmse_m(
    const double & _arg)
  {
    this->alignment_rmse_m = _arg;
    return *this;
  }
  Type & set__r_gps_lidar(
    const std::array<double, 9> & _arg)
  {
    this->r_gps_lidar = _arg;
    return *this;
  }
  Type & set__t_gps_lidar(
    const std::array<double, 3> & _arg)
  {
    this->t_gps_lidar = _arg;
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
    automap_pro::srv::TriggerGpsAlign_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const automap_pro::srv::TriggerGpsAlign_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<automap_pro::srv::TriggerGpsAlign_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<automap_pro::srv::TriggerGpsAlign_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      automap_pro::srv::TriggerGpsAlign_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<automap_pro::srv::TriggerGpsAlign_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      automap_pro::srv::TriggerGpsAlign_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<automap_pro::srv::TriggerGpsAlign_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<automap_pro::srv::TriggerGpsAlign_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<automap_pro::srv::TriggerGpsAlign_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__automap_pro__srv__TriggerGpsAlign_Response
    std::shared_ptr<automap_pro::srv::TriggerGpsAlign_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__automap_pro__srv__TriggerGpsAlign_Response
    std::shared_ptr<automap_pro::srv::TriggerGpsAlign_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TriggerGpsAlign_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->alignment_rmse_m != other.alignment_rmse_m) {
      return false;
    }
    if (this->r_gps_lidar != other.r_gps_lidar) {
      return false;
    }
    if (this->t_gps_lidar != other.t_gps_lidar) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const TriggerGpsAlign_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TriggerGpsAlign_Response_

// alias to use template instance with default allocator
using TriggerGpsAlign_Response =
  automap_pro::srv::TriggerGpsAlign_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace automap_pro

namespace automap_pro
{

namespace srv
{

struct TriggerGpsAlign
{
  using Request = automap_pro::srv::TriggerGpsAlign_Request;
  using Response = automap_pro::srv::TriggerGpsAlign_Response;
};

}  // namespace srv

}  // namespace automap_pro

#endif  // AUTOMAP_PRO__SRV__DETAIL__TRIGGER_GPS_ALIGN__STRUCT_HPP_
