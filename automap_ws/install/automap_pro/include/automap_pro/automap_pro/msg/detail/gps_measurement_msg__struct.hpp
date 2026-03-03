// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from automap_pro:msg/GPSMeasurementMsg.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__MSG__DETAIL__GPS_MEASUREMENT_MSG__STRUCT_HPP_
#define AUTOMAP_PRO__MSG__DETAIL__GPS_MEASUREMENT_MSG__STRUCT_HPP_

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
# define DEPRECATED__automap_pro__msg__GPSMeasurementMsg __attribute__((deprecated))
#else
# define DEPRECATED__automap_pro__msg__GPSMeasurementMsg __declspec(deprecated)
#endif

namespace automap_pro
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GPSMeasurementMsg_
{
  using Type = GPSMeasurementMsg_<ContainerAllocator>;

  explicit GPSMeasurementMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 3>::iterator, double>(this->position_enu.begin(), this->position_enu.end(), 0.0);
      this->quality = 0l;
      this->hdop = 0.0f;
      this->num_satellites = 0l;
      std::fill<typename std::array<double, 9>::iterator, double>(this->covariance.begin(), this->covariance.end(), 0.0);
      this->is_valid = false;
      this->latitude = 0.0;
      this->longitude = 0.0;
      this->altitude = 0.0;
    }
  }

  explicit GPSMeasurementMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    position_enu(_alloc),
    covariance(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 3>::iterator, double>(this->position_enu.begin(), this->position_enu.end(), 0.0);
      this->quality = 0l;
      this->hdop = 0.0f;
      this->num_satellites = 0l;
      std::fill<typename std::array<double, 9>::iterator, double>(this->covariance.begin(), this->covariance.end(), 0.0);
      this->is_valid = false;
      this->latitude = 0.0;
      this->longitude = 0.0;
      this->altitude = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _position_enu_type =
    std::array<double, 3>;
  _position_enu_type position_enu;
  using _quality_type =
    int32_t;
  _quality_type quality;
  using _hdop_type =
    float;
  _hdop_type hdop;
  using _num_satellites_type =
    int32_t;
  _num_satellites_type num_satellites;
  using _covariance_type =
    std::array<double, 9>;
  _covariance_type covariance;
  using _is_valid_type =
    bool;
  _is_valid_type is_valid;
  using _latitude_type =
    double;
  _latitude_type latitude;
  using _longitude_type =
    double;
  _longitude_type longitude;
  using _altitude_type =
    double;
  _altitude_type altitude;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__position_enu(
    const std::array<double, 3> & _arg)
  {
    this->position_enu = _arg;
    return *this;
  }
  Type & set__quality(
    const int32_t & _arg)
  {
    this->quality = _arg;
    return *this;
  }
  Type & set__hdop(
    const float & _arg)
  {
    this->hdop = _arg;
    return *this;
  }
  Type & set__num_satellites(
    const int32_t & _arg)
  {
    this->num_satellites = _arg;
    return *this;
  }
  Type & set__covariance(
    const std::array<double, 9> & _arg)
  {
    this->covariance = _arg;
    return *this;
  }
  Type & set__is_valid(
    const bool & _arg)
  {
    this->is_valid = _arg;
    return *this;
  }
  Type & set__latitude(
    const double & _arg)
  {
    this->latitude = _arg;
    return *this;
  }
  Type & set__longitude(
    const double & _arg)
  {
    this->longitude = _arg;
    return *this;
  }
  Type & set__altitude(
    const double & _arg)
  {
    this->altitude = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    automap_pro::msg::GPSMeasurementMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const automap_pro::msg::GPSMeasurementMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<automap_pro::msg::GPSMeasurementMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<automap_pro::msg::GPSMeasurementMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      automap_pro::msg::GPSMeasurementMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<automap_pro::msg::GPSMeasurementMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      automap_pro::msg::GPSMeasurementMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<automap_pro::msg::GPSMeasurementMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<automap_pro::msg::GPSMeasurementMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<automap_pro::msg::GPSMeasurementMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__automap_pro__msg__GPSMeasurementMsg
    std::shared_ptr<automap_pro::msg::GPSMeasurementMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__automap_pro__msg__GPSMeasurementMsg
    std::shared_ptr<automap_pro::msg::GPSMeasurementMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GPSMeasurementMsg_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->position_enu != other.position_enu) {
      return false;
    }
    if (this->quality != other.quality) {
      return false;
    }
    if (this->hdop != other.hdop) {
      return false;
    }
    if (this->num_satellites != other.num_satellites) {
      return false;
    }
    if (this->covariance != other.covariance) {
      return false;
    }
    if (this->is_valid != other.is_valid) {
      return false;
    }
    if (this->latitude != other.latitude) {
      return false;
    }
    if (this->longitude != other.longitude) {
      return false;
    }
    if (this->altitude != other.altitude) {
      return false;
    }
    return true;
  }
  bool operator!=(const GPSMeasurementMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GPSMeasurementMsg_

// alias to use template instance with default allocator
using GPSMeasurementMsg =
  automap_pro::msg::GPSMeasurementMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace automap_pro

#endif  // AUTOMAP_PRO__MSG__DETAIL__GPS_MEASUREMENT_MSG__STRUCT_HPP_
