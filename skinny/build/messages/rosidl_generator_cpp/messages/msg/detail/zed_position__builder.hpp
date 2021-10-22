// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from messages:msg/ZedPosition.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__ZED_POSITION__BUILDER_HPP_
#define MESSAGES__MSG__DETAIL__ZED_POSITION__BUILDER_HPP_

#include "messages/msg/detail/zed_position__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace messages
{

namespace msg
{

namespace builder
{

class Init_ZedPosition_aruco_visible
{
public:
  explicit Init_ZedPosition_aruco_visible(::messages::msg::ZedPosition & msg)
  : msg_(msg)
  {}
  ::messages::msg::ZedPosition aruco_visible(::messages::msg::ZedPosition::_aruco_visible_type arg)
  {
    msg_.aruco_visible = std::move(arg);
    return std::move(msg_);
  }

private:
  ::messages::msg::ZedPosition msg_;
};

class Init_ZedPosition_ow
{
public:
  explicit Init_ZedPosition_ow(::messages::msg::ZedPosition & msg)
  : msg_(msg)
  {}
  Init_ZedPosition_aruco_visible ow(::messages::msg::ZedPosition::_ow_type arg)
  {
    msg_.ow = std::move(arg);
    return Init_ZedPosition_aruco_visible(msg_);
  }

private:
  ::messages::msg::ZedPosition msg_;
};

class Init_ZedPosition_oz
{
public:
  explicit Init_ZedPosition_oz(::messages::msg::ZedPosition & msg)
  : msg_(msg)
  {}
  Init_ZedPosition_ow oz(::messages::msg::ZedPosition::_oz_type arg)
  {
    msg_.oz = std::move(arg);
    return Init_ZedPosition_ow(msg_);
  }

private:
  ::messages::msg::ZedPosition msg_;
};

class Init_ZedPosition_oy
{
public:
  explicit Init_ZedPosition_oy(::messages::msg::ZedPosition & msg)
  : msg_(msg)
  {}
  Init_ZedPosition_oz oy(::messages::msg::ZedPosition::_oy_type arg)
  {
    msg_.oy = std::move(arg);
    return Init_ZedPosition_oz(msg_);
  }

private:
  ::messages::msg::ZedPosition msg_;
};

class Init_ZedPosition_ox
{
public:
  explicit Init_ZedPosition_ox(::messages::msg::ZedPosition & msg)
  : msg_(msg)
  {}
  Init_ZedPosition_oy ox(::messages::msg::ZedPosition::_ox_type arg)
  {
    msg_.ox = std::move(arg);
    return Init_ZedPosition_oy(msg_);
  }

private:
  ::messages::msg::ZedPosition msg_;
};

class Init_ZedPosition_z
{
public:
  explicit Init_ZedPosition_z(::messages::msg::ZedPosition & msg)
  : msg_(msg)
  {}
  Init_ZedPosition_ox z(::messages::msg::ZedPosition::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_ZedPosition_ox(msg_);
  }

private:
  ::messages::msg::ZedPosition msg_;
};

class Init_ZedPosition_y
{
public:
  explicit Init_ZedPosition_y(::messages::msg::ZedPosition & msg)
  : msg_(msg)
  {}
  Init_ZedPosition_z y(::messages::msg::ZedPosition::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_ZedPosition_z(msg_);
  }

private:
  ::messages::msg::ZedPosition msg_;
};

class Init_ZedPosition_x
{
public:
  Init_ZedPosition_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ZedPosition_y x(::messages::msg::ZedPosition::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_ZedPosition_y(msg_);
  }

private:
  ::messages::msg::ZedPosition msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::messages::msg::ZedPosition>()
{
  return messages::msg::builder::Init_ZedPosition_x();
}

}  // namespace messages

#endif  // MESSAGES__MSG__DETAIL__ZED_POSITION__BUILDER_HPP_
