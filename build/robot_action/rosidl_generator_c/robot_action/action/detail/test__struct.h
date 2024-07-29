// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_action:action/Test.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_ACTION__ACTION__DETAIL__TEST__STRUCT_H_
#define ROBOT_ACTION__ACTION__DETAIL__TEST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/Test in the package robot_action.
typedef struct robot_action__action__Test_Goal
{
  rosidl_runtime_c__String message;
} robot_action__action__Test_Goal;

// Struct for a sequence of robot_action__action__Test_Goal.
typedef struct robot_action__action__Test_Goal__Sequence
{
  robot_action__action__Test_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_action__action__Test_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'response'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/Test in the package robot_action.
typedef struct robot_action__action__Test_Result
{
  rosidl_runtime_c__String response;
} robot_action__action__Test_Result;

// Struct for a sequence of robot_action__action__Test_Result.
typedef struct robot_action__action__Test_Result__Sequence
{
  robot_action__action__Test_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_action__action__Test_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'feedback'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/Test in the package robot_action.
typedef struct robot_action__action__Test_Feedback
{
  rosidl_runtime_c__String feedback;
} robot_action__action__Test_Feedback;

// Struct for a sequence of robot_action__action__Test_Feedback.
typedef struct robot_action__action__Test_Feedback__Sequence
{
  robot_action__action__Test_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_action__action__Test_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "robot_action/action/detail/test__struct.h"

/// Struct defined in action/Test in the package robot_action.
typedef struct robot_action__action__Test_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  robot_action__action__Test_Goal goal;
} robot_action__action__Test_SendGoal_Request;

// Struct for a sequence of robot_action__action__Test_SendGoal_Request.
typedef struct robot_action__action__Test_SendGoal_Request__Sequence
{
  robot_action__action__Test_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_action__action__Test_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/Test in the package robot_action.
typedef struct robot_action__action__Test_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} robot_action__action__Test_SendGoal_Response;

// Struct for a sequence of robot_action__action__Test_SendGoal_Response.
typedef struct robot_action__action__Test_SendGoal_Response__Sequence
{
  robot_action__action__Test_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_action__action__Test_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/Test in the package robot_action.
typedef struct robot_action__action__Test_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} robot_action__action__Test_GetResult_Request;

// Struct for a sequence of robot_action__action__Test_GetResult_Request.
typedef struct robot_action__action__Test_GetResult_Request__Sequence
{
  robot_action__action__Test_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_action__action__Test_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "robot_action/action/detail/test__struct.h"

/// Struct defined in action/Test in the package robot_action.
typedef struct robot_action__action__Test_GetResult_Response
{
  int8_t status;
  robot_action__action__Test_Result result;
} robot_action__action__Test_GetResult_Response;

// Struct for a sequence of robot_action__action__Test_GetResult_Response.
typedef struct robot_action__action__Test_GetResult_Response__Sequence
{
  robot_action__action__Test_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_action__action__Test_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "robot_action/action/detail/test__struct.h"

/// Struct defined in action/Test in the package robot_action.
typedef struct robot_action__action__Test_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  robot_action__action__Test_Feedback feedback;
} robot_action__action__Test_FeedbackMessage;

// Struct for a sequence of robot_action__action__Test_FeedbackMessage.
typedef struct robot_action__action__Test_FeedbackMessage__Sequence
{
  robot_action__action__Test_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_action__action__Test_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_ACTION__ACTION__DETAIL__TEST__STRUCT_H_
