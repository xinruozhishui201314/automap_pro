// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from automap_pro:msg/GPSMeasurementMsg.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__MSG__DETAIL__GPS_MEASUREMENT_MSG__BUILDER_HPP_
#define AUTOMAP_PRO__MSG__DETAIL__GPS_MEASUREMENT_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "automap_pro/msg/detail/gps_measurement_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace automap_pro
{

namespace msg
{

namespace builder
{

class Init_GPSMeasurementMsg_altitude
{
public:
  explicit Init_GPSMeasurementMsg_altitude(::automap_pro::msg::GPSMeasurementMsg & msg)
  : msg_(msg)
  {}
  ::automap_pro::msg::GPSMeasurementMsg altitude(::automap_pro::msg::GPSMeasurementMsg::_altitude_type arg)
  {
    msg_.altitude = std::move(arg);
    return std::move(msg_);
  }

private:
  ::automap_pro::msg::GPSMeasurementMsg msg_;
};

class Init_GPSMeasurementMsg_longitude
{
public:
  explicit Init_GPSMeasurementMsg_longitude(::automap_pro::msg::GPSMeasurementMsg & msg)
  : msg_(msg)
  {}
  Init_GPSMeasurementMsg_altitude longitude(::automap_pro::msg::GPSMeasurementMsg::_longitude_type arg)
  {
    msg_.longitude = std::move(arg);
    return Init_GPSMeasurementMsg_altitude(msg_);
  }

private:
  ::automap_pro::msg::GPSMeasurementMsg msg_;
};

class Init_GPSMeasurementMsg_latitude
{
public:
  explicit Init_GPSMeasurementMsg_latitude(::automap_pro::msg::GPSMeasurementMsg & msg)
  : msg_(msg)
  {}
  Init_GPSMeasurementMsg_longitude latitude(::automap_pro::msg::GPSMeasurementMsg::_latitude_type arg)
  {
    msg_.latitude = std::move(arg);
    return Init_GPSMeasurementMsg_longitude(msg_);
  }

private:
  ::automap_pro::msg::GPSMeasurementMsg msg_;
};

class Init_GPSMeasurementMsg_is_valid
{
public:
  explicit Init_GPSMeasurementMsg_is_valid(::automap_pro::msg::GPSMeasurementMsg & msg)
  : msg_(msg)
  {}
  Init_GPSMeasurementMsg_latitude is_valid(::automap_pro::msg::GPSMeasurementMsg::_is_valid_type arg)
  {
    msg_.is_valid = std::move(arg);
    return Init_GPSMeasurementMsg_latitude(msg_);
  }

private:
  ::automap_pro::msg::GPSMeasurementMsg msg_;
};

class Init_GPSMeasurementMsg_covariance
{
public:
  explicit Init_GPSMeasurementMsg_covariance(::automap_pro::msg::GPSMeasurementMsg & msg)
  : msg_(msg)
  {}
  Init_GPSMeasurementMsg_is_valid covariance(::automap_pro::msg::GPSMeasurementMsg::_covariance_type arg)
  {
    msg_.covariance = std::move(arg);
    return Init_GPSMeasurementMsg_is_valid(msg_);
  }

private:
  ::automap_pro::msg::GPSMeasurementMsg msg_;
};

class Init_GPSMeasurementMsg_num_satellites
{
public:
  explicit Init_GPSMeasurementMsg_num_satellites(::automap_pro::msg::GPSMeasurementMsg & msg)
  : msg_(msg)
  {}
  Init_GPSMeasurementMsg_covariance num_satellites(::automap_pro::msg::GPSMeasurementMsg::_num_satellites_type arg)
  {
    msg_.num_satellites = std::move(arg);
    return Init_GPSMeasurementMsg_covariance(msg_);
  }

private:
  ::automap_pro::msg::GPSMeasurementMsg msg_;
};

class Init_GPSMeasurementMsg_hdop
{
public:
  explicit Init_GPSMeasurementMsg_hdop(::automap_pro::msg::GPSMeasurementMsg & msg)
  : msg_(msg)
  {}
  Init_GPSMeasurementMsg_num_satellites hdop(::automap_pro::msg::GPSMeasurementMsg::_hdop_type arg)
  {
    msg_.hdop = std::move(arg);
    return Init_GPSMeasurementMsg_num_satellites(msg_);
  }

private:
  ::automap_pro::msg::GPSMeasurementMsg msg_;
};

class Init_GPSMeasurementMsg_quality
{
public:
  explicit Init_GPSMeasurementMsg_quality(::automap_pro::msg::GPSMeasurementMsg & msg)
  : msg_(msg)
  {}
  Init_GPSMeasurementMsg_hdop quality(::automap_pro::msg::GPSMeasurementMsg::_quality_type arg)
  {
    msg_.quality = std::move(arg);
    return Init_GPSMeasurementMsg_hdop(msg_);
  }

private:
  ::automap_pro::msg::GPSMeasurementMsg msg_;
};

class Init_GPSMeasurementMsg_position_enu
{
public:
  explicit Init_GPSMeasurementMsg_position_enu(::automap_pro::msg::GPSMeasurementMsg & msg)
  : msg_(msg)
  {}
  Init_GPSMeasurementMsg_quality position_enu(::automap_pro::msg::GPSMeasurementMsg::_position_enu_type arg)
  {
    msg_.position_enu = std::move(arg);
    return Init_GPSMeasurementMsg_quality(msg_);
  }

private:
  ::automap_pro::msg::GPSMeasurementMsg msg_;
};

class Init_GPSMeasurementMsg_header
{
public:
  Init_GPSMeasurementMsg_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GPSMeasurementMsg_position_enu header(::automap_pro::msg::GPSMeasurementMsg::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_GPSMeasurementMsg_position_enu(msg_);
  }

private:
  ::automap_pro::msg::GPSMeasurementMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::automap_pro::msg::GPSMeasurementMsg>()
{
  return automap_pro::msg::builder::Init_GPSMeasurementMsg_header();
}

}  // namespace automap_pro

#endif  // AUTOMAP_PRO__MSG__DETAIL__GPS_MEASUREMENT_MSG__BUILDER_HPP_
