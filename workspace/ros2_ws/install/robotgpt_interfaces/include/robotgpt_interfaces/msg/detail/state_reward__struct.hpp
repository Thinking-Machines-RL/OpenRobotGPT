// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robotgpt_interfaces:msg/StateReward.idl
// generated code does not contain a copyright notice

#ifndef ROBOTGPT_INTERFACES__MSG__DETAIL__STATE_REWARD__STRUCT_HPP_
#define ROBOTGPT_INTERFACES__MSG__DETAIL__STATE_REWARD__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__robotgpt_interfaces__msg__StateReward __attribute__((deprecated))
#else
# define DEPRECATED__robotgpt_interfaces__msg__StateReward __declspec(deprecated)
#endif

namespace robotgpt_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct StateReward_
{
  using Type = StateReward_<ContainerAllocator>;

  explicit StateReward_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->reward = 0.0f;
      this->terminal = false;
    }
  }

  explicit StateReward_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->reward = 0.0f;
      this->terminal = false;
    }
  }

  // field types and members
  using _state_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _state_type state;
  using _info_keys_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other>;
  _info_keys_type info_keys;
  using _info_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _info_type info;
  using _reward_type =
    float;
  _reward_type reward;
  using _terminal_type =
    bool;
  _terminal_type terminal;

  // setters for named parameter idiom
  Type & set__state(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->state = _arg;
    return *this;
  }
  Type & set__info_keys(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other> & _arg)
  {
    this->info_keys = _arg;
    return *this;
  }
  Type & set__info(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__reward(
    const float & _arg)
  {
    this->reward = _arg;
    return *this;
  }
  Type & set__terminal(
    const bool & _arg)
  {
    this->terminal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robotgpt_interfaces::msg::StateReward_<ContainerAllocator> *;
  using ConstRawPtr =
    const robotgpt_interfaces::msg::StateReward_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robotgpt_interfaces::msg::StateReward_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robotgpt_interfaces::msg::StateReward_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robotgpt_interfaces::msg::StateReward_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robotgpt_interfaces::msg::StateReward_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robotgpt_interfaces::msg::StateReward_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robotgpt_interfaces::msg::StateReward_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robotgpt_interfaces::msg::StateReward_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robotgpt_interfaces::msg::StateReward_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robotgpt_interfaces__msg__StateReward
    std::shared_ptr<robotgpt_interfaces::msg::StateReward_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robotgpt_interfaces__msg__StateReward
    std::shared_ptr<robotgpt_interfaces::msg::StateReward_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StateReward_ & other) const
  {
    if (this->state != other.state) {
      return false;
    }
    if (this->info_keys != other.info_keys) {
      return false;
    }
    if (this->info != other.info) {
      return false;
    }
    if (this->reward != other.reward) {
      return false;
    }
    if (this->terminal != other.terminal) {
      return false;
    }
    return true;
  }
  bool operator!=(const StateReward_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StateReward_

// alias to use template instance with default allocator
using StateReward =
  robotgpt_interfaces::msg::StateReward_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robotgpt_interfaces

#endif  // ROBOTGPT_INTERFACES__MSG__DETAIL__STATE_REWARD__STRUCT_HPP_
