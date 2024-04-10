// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from messages:srv/CodeExecution.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__SRV__DETAIL__CODE_EXECUTION__BUILDER_HPP_
#define MESSAGES__SRV__DETAIL__CODE_EXECUTION__BUILDER_HPP_

#include "messages/srv/detail/code_execution__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace messages
{

namespace srv
{

namespace builder
{

class Init_CodeExecution_Request_evaluation_code
{
public:
  explicit Init_CodeExecution_Request_evaluation_code(::messages::srv::CodeExecution_Request & msg)
  : msg_(msg)
  {}
  ::messages::srv::CodeExecution_Request evaluation_code(::messages::srv::CodeExecution_Request::_evaluation_code_type arg)
  {
    msg_.evaluation_code = std::move(arg);
    return std::move(msg_);
  }

private:
  ::messages::srv::CodeExecution_Request msg_;
};

class Init_CodeExecution_Request_code
{
public:
  Init_CodeExecution_Request_code()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CodeExecution_Request_evaluation_code code(::messages::srv::CodeExecution_Request::_code_type arg)
  {
    msg_.code = std::move(arg);
    return Init_CodeExecution_Request_evaluation_code(msg_);
  }

private:
  ::messages::srv::CodeExecution_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::messages::srv::CodeExecution_Request>()
{
  return messages::srv::builder::Init_CodeExecution_Request_code();
}

}  // namespace messages


namespace messages
{

namespace srv
{

namespace builder
{

class Init_CodeExecution_Response_completion_flag
{
public:
  Init_CodeExecution_Response_completion_flag()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::messages::srv::CodeExecution_Response completion_flag(::messages::srv::CodeExecution_Response::_completion_flag_type arg)
  {
    msg_.completion_flag = std::move(arg);
    return std::move(msg_);
  }

private:
  ::messages::srv::CodeExecution_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::messages::srv::CodeExecution_Response>()
{
  return messages::srv::builder::Init_CodeExecution_Response_completion_flag();
}

}  // namespace messages

#endif  // MESSAGES__SRV__DETAIL__CODE_EXECUTION__BUILDER_HPP_
