// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from messages:srv/CodeExecution.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__SRV__DETAIL__CODE_EXECUTION__TRAITS_HPP_
#define MESSAGES__SRV__DETAIL__CODE_EXECUTION__TRAITS_HPP_

#include "messages/srv/detail/code_execution__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<messages::srv::CodeExecution_Request>()
{
  return "messages::srv::CodeExecution_Request";
}

template<>
inline const char * name<messages::srv::CodeExecution_Request>()
{
  return "messages/srv/CodeExecution_Request";
}

template<>
struct has_fixed_size<messages::srv::CodeExecution_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<messages::srv::CodeExecution_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<messages::srv::CodeExecution_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<messages::srv::CodeExecution_Response>()
{
  return "messages::srv::CodeExecution_Response";
}

template<>
inline const char * name<messages::srv::CodeExecution_Response>()
{
  return "messages/srv/CodeExecution_Response";
}

template<>
struct has_fixed_size<messages::srv::CodeExecution_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<messages::srv::CodeExecution_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<messages::srv::CodeExecution_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<messages::srv::CodeExecution>()
{
  return "messages::srv::CodeExecution";
}

template<>
inline const char * name<messages::srv::CodeExecution>()
{
  return "messages/srv/CodeExecution";
}

template<>
struct has_fixed_size<messages::srv::CodeExecution>
  : std::integral_constant<
    bool,
    has_fixed_size<messages::srv::CodeExecution_Request>::value &&
    has_fixed_size<messages::srv::CodeExecution_Response>::value
  >
{
};

template<>
struct has_bounded_size<messages::srv::CodeExecution>
  : std::integral_constant<
    bool,
    has_bounded_size<messages::srv::CodeExecution_Request>::value &&
    has_bounded_size<messages::srv::CodeExecution_Response>::value
  >
{
};

template<>
struct is_service<messages::srv::CodeExecution>
  : std::true_type
{
};

template<>
struct is_service_request<messages::srv::CodeExecution_Request>
  : std::true_type
{
};

template<>
struct is_service_response<messages::srv::CodeExecution_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MESSAGES__SRV__DETAIL__CODE_EXECUTION__TRAITS_HPP_
