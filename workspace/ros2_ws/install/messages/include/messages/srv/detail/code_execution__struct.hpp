// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from messages:srv/CodeExecution.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__SRV__DETAIL__CODE_EXECUTION__STRUCT_HPP_
#define MESSAGES__SRV__DETAIL__CODE_EXECUTION__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__messages__srv__CodeExecution_Request __attribute__((deprecated))
#else
# define DEPRECATED__messages__srv__CodeExecution_Request __declspec(deprecated)
#endif

namespace messages
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CodeExecution_Request_
{
  using Type = CodeExecution_Request_<ContainerAllocator>;

  explicit CodeExecution_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->code = "";
      this->evaluation_code = "";
    }
  }

  explicit CodeExecution_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : code(_alloc),
    evaluation_code(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->code = "";
      this->evaluation_code = "";
    }
  }

  // field types and members
  using _code_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _code_type code;
  using _evaluation_code_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _evaluation_code_type evaluation_code;

  // setters for named parameter idiom
  Type & set__code(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->code = _arg;
    return *this;
  }
  Type & set__evaluation_code(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->evaluation_code = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    messages::srv::CodeExecution_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const messages::srv::CodeExecution_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<messages::srv::CodeExecution_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<messages::srv::CodeExecution_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      messages::srv::CodeExecution_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<messages::srv::CodeExecution_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      messages::srv::CodeExecution_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<messages::srv::CodeExecution_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<messages::srv::CodeExecution_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<messages::srv::CodeExecution_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__messages__srv__CodeExecution_Request
    std::shared_ptr<messages::srv::CodeExecution_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__messages__srv__CodeExecution_Request
    std::shared_ptr<messages::srv::CodeExecution_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CodeExecution_Request_ & other) const
  {
    if (this->code != other.code) {
      return false;
    }
    if (this->evaluation_code != other.evaluation_code) {
      return false;
    }
    return true;
  }
  bool operator!=(const CodeExecution_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CodeExecution_Request_

// alias to use template instance with default allocator
using CodeExecution_Request =
  messages::srv::CodeExecution_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace messages


#ifndef _WIN32
# define DEPRECATED__messages__srv__CodeExecution_Response __attribute__((deprecated))
#else
# define DEPRECATED__messages__srv__CodeExecution_Response __declspec(deprecated)
#endif

namespace messages
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CodeExecution_Response_
{
  using Type = CodeExecution_Response_<ContainerAllocator>;

  explicit CodeExecution_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->completion_flag = false;
    }
  }

  explicit CodeExecution_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->completion_flag = false;
    }
  }

  // field types and members
  using _completion_flag_type =
    bool;
  _completion_flag_type completion_flag;

  // setters for named parameter idiom
  Type & set__completion_flag(
    const bool & _arg)
  {
    this->completion_flag = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    messages::srv::CodeExecution_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const messages::srv::CodeExecution_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<messages::srv::CodeExecution_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<messages::srv::CodeExecution_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      messages::srv::CodeExecution_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<messages::srv::CodeExecution_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      messages::srv::CodeExecution_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<messages::srv::CodeExecution_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<messages::srv::CodeExecution_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<messages::srv::CodeExecution_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__messages__srv__CodeExecution_Response
    std::shared_ptr<messages::srv::CodeExecution_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__messages__srv__CodeExecution_Response
    std::shared_ptr<messages::srv::CodeExecution_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CodeExecution_Response_ & other) const
  {
    if (this->completion_flag != other.completion_flag) {
      return false;
    }
    return true;
  }
  bool operator!=(const CodeExecution_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CodeExecution_Response_

// alias to use template instance with default allocator
using CodeExecution_Response =
  messages::srv::CodeExecution_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace messages

namespace messages
{

namespace srv
{

struct CodeExecution
{
  using Request = messages::srv::CodeExecution_Request;
  using Response = messages::srv::CodeExecution_Response;
};

}  // namespace srv

}  // namespace messages

#endif  // MESSAGES__SRV__DETAIL__CODE_EXECUTION__STRUCT_HPP_
