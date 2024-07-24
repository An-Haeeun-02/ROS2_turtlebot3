// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_action:action/Test.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_ACTION__ACTION__DETAIL__TEST__BUILDER_HPP_
#define ROBOT_ACTION__ACTION__DETAIL__TEST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_action/action/detail/test__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_action
{

namespace action
{

namespace builder
{

class Init_Test_Goal_message
{
public:
  Init_Test_Goal_message()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_action::action::Test_Goal message(::robot_action::action::Test_Goal::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_action::action::Test_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_action::action::Test_Goal>()
{
  return robot_action::action::builder::Init_Test_Goal_message();
}

}  // namespace robot_action


namespace robot_action
{

namespace action
{

namespace builder
{

class Init_Test_Result_response
{
public:
  Init_Test_Result_response()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_action::action::Test_Result response(::robot_action::action::Test_Result::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_action::action::Test_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_action::action::Test_Result>()
{
  return robot_action::action::builder::Init_Test_Result_response();
}

}  // namespace robot_action


namespace robot_action
{

namespace action
{

namespace builder
{

class Init_Test_Feedback_feedback
{
public:
  Init_Test_Feedback_feedback()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_action::action::Test_Feedback feedback(::robot_action::action::Test_Feedback::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_action::action::Test_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_action::action::Test_Feedback>()
{
  return robot_action::action::builder::Init_Test_Feedback_feedback();
}

}  // namespace robot_action


namespace robot_action
{

namespace action
{

namespace builder
{

class Init_Test_SendGoal_Request_goal
{
public:
  explicit Init_Test_SendGoal_Request_goal(::robot_action::action::Test_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::robot_action::action::Test_SendGoal_Request goal(::robot_action::action::Test_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_action::action::Test_SendGoal_Request msg_;
};

class Init_Test_SendGoal_Request_goal_id
{
public:
  Init_Test_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Test_SendGoal_Request_goal goal_id(::robot_action::action::Test_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Test_SendGoal_Request_goal(msg_);
  }

private:
  ::robot_action::action::Test_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_action::action::Test_SendGoal_Request>()
{
  return robot_action::action::builder::Init_Test_SendGoal_Request_goal_id();
}

}  // namespace robot_action


namespace robot_action
{

namespace action
{

namespace builder
{

class Init_Test_SendGoal_Response_stamp
{
public:
  explicit Init_Test_SendGoal_Response_stamp(::robot_action::action::Test_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::robot_action::action::Test_SendGoal_Response stamp(::robot_action::action::Test_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_action::action::Test_SendGoal_Response msg_;
};

class Init_Test_SendGoal_Response_accepted
{
public:
  Init_Test_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Test_SendGoal_Response_stamp accepted(::robot_action::action::Test_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_Test_SendGoal_Response_stamp(msg_);
  }

private:
  ::robot_action::action::Test_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_action::action::Test_SendGoal_Response>()
{
  return robot_action::action::builder::Init_Test_SendGoal_Response_accepted();
}

}  // namespace robot_action


namespace robot_action
{

namespace action
{

namespace builder
{

class Init_Test_GetResult_Request_goal_id
{
public:
  Init_Test_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_action::action::Test_GetResult_Request goal_id(::robot_action::action::Test_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_action::action::Test_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_action::action::Test_GetResult_Request>()
{
  return robot_action::action::builder::Init_Test_GetResult_Request_goal_id();
}

}  // namespace robot_action


namespace robot_action
{

namespace action
{

namespace builder
{

class Init_Test_GetResult_Response_result
{
public:
  explicit Init_Test_GetResult_Response_result(::robot_action::action::Test_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::robot_action::action::Test_GetResult_Response result(::robot_action::action::Test_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_action::action::Test_GetResult_Response msg_;
};

class Init_Test_GetResult_Response_status
{
public:
  Init_Test_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Test_GetResult_Response_result status(::robot_action::action::Test_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Test_GetResult_Response_result(msg_);
  }

private:
  ::robot_action::action::Test_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_action::action::Test_GetResult_Response>()
{
  return robot_action::action::builder::Init_Test_GetResult_Response_status();
}

}  // namespace robot_action


namespace robot_action
{

namespace action
{

namespace builder
{

class Init_Test_FeedbackMessage_feedback
{
public:
  explicit Init_Test_FeedbackMessage_feedback(::robot_action::action::Test_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::robot_action::action::Test_FeedbackMessage feedback(::robot_action::action::Test_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_action::action::Test_FeedbackMessage msg_;
};

class Init_Test_FeedbackMessage_goal_id
{
public:
  Init_Test_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Test_FeedbackMessage_feedback goal_id(::robot_action::action::Test_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Test_FeedbackMessage_feedback(msg_);
  }

private:
  ::robot_action::action::Test_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_action::action::Test_FeedbackMessage>()
{
  return robot_action::action::builder::Init_Test_FeedbackMessage_goal_id();
}

}  // namespace robot_action

#endif  // ROBOT_ACTION__ACTION__DETAIL__TEST__BUILDER_HPP_
