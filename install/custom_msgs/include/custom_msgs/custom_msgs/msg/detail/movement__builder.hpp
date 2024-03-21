// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/Movement.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__MOVEMENT__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__MOVEMENT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/msg/detail/movement__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_Movement_acceleration
{
public:
  explicit Init_Movement_acceleration(::custom_msgs::msg::Movement & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::Movement acceleration(::custom_msgs::msg::Movement::_acceleration_type arg)
  {
    msg_.acceleration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::Movement msg_;
};

class Init_Movement_velocity
{
public:
  Init_Movement_velocity()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Movement_acceleration velocity(::custom_msgs::msg::Movement::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_Movement_acceleration(msg_);
  }

private:
  ::custom_msgs::msg::Movement msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::Movement>()
{
  return custom_msgs::msg::builder::Init_Movement_velocity();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__MOVEMENT__BUILDER_HPP_
