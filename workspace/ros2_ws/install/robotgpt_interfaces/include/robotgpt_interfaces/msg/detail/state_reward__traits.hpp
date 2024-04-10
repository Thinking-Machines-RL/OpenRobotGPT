// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robotgpt_interfaces:msg/StateReward.idl
// generated code does not contain a copyright notice

#ifndef ROBOTGPT_INTERFACES__MSG__DETAIL__STATE_REWARD__TRAITS_HPP_
#define ROBOTGPT_INTERFACES__MSG__DETAIL__STATE_REWARD__TRAITS_HPP_

#include "robotgpt_interfaces/msg/detail/state_reward__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robotgpt_interfaces::msg::StateReward>()
{
  return "robotgpt_interfaces::msg::StateReward";
}

template<>
inline const char * name<robotgpt_interfaces::msg::StateReward>()
{
  return "robotgpt_interfaces/msg/StateReward";
}

template<>
struct has_fixed_size<robotgpt_interfaces::msg::StateReward>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robotgpt_interfaces::msg::StateReward>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robotgpt_interfaces::msg::StateReward>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOTGPT_INTERFACES__MSG__DETAIL__STATE_REWARD__TRAITS_HPP_
