// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from limo_msgs:msg/LimoStatus.idl
// generated code does not contain a copyright notice

#ifndef LIMO_MSGS__MSG__DETAIL__LIMO_STATUS__TRAITS_HPP_
#define LIMO_MSGS__MSG__DETAIL__LIMO_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "limo_msgs/msg/detail/limo_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace limo_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const LimoStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: vehicle_state
  {
    out << "vehicle_state: ";
    rosidl_generator_traits::value_to_yaml(msg.vehicle_state, out);
    out << ", ";
  }

  // member: control_mode
  {
    out << "control_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.control_mode, out);
    out << ", ";
  }

  // member: battery_voltage
  {
    out << "battery_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_voltage, out);
    out << ", ";
  }

  // member: error_code
  {
    out << "error_code: ";
    rosidl_generator_traits::value_to_yaml(msg.error_code, out);
    out << ", ";
  }

  // member: motion_mode
  {
    out << "motion_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.motion_mode, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LimoStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: vehicle_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vehicle_state: ";
    rosidl_generator_traits::value_to_yaml(msg.vehicle_state, out);
    out << "\n";
  }

  // member: control_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "control_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.control_mode, out);
    out << "\n";
  }

  // member: battery_voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_voltage, out);
    out << "\n";
  }

  // member: error_code
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_code: ";
    rosidl_generator_traits::value_to_yaml(msg.error_code, out);
    out << "\n";
  }

  // member: motion_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motion_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.motion_mode, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LimoStatus & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace limo_msgs

namespace rosidl_generator_traits
{

[[deprecated("use limo_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const limo_msgs::msg::LimoStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  limo_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use limo_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const limo_msgs::msg::LimoStatus & msg)
{
  return limo_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<limo_msgs::msg::LimoStatus>()
{
  return "limo_msgs::msg::LimoStatus";
}

template<>
inline const char * name<limo_msgs::msg::LimoStatus>()
{
  return "limo_msgs/msg/LimoStatus";
}

template<>
struct has_fixed_size<limo_msgs::msg::LimoStatus>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<limo_msgs::msg::LimoStatus>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<limo_msgs::msg::LimoStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // LIMO_MSGS__MSG__DETAIL__LIMO_STATUS__TRAITS_HPP_
