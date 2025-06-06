// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from robot_action:action/Test.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_ACTION__ACTION__DETAIL__TEST__FUNCTIONS_H_
#define ROBOT_ACTION__ACTION__DETAIL__TEST__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "robot_action/msg/rosidl_generator_c__visibility_control.h"

#include "robot_action/action/detail/test__struct.h"

/// Initialize action/Test message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * robot_action__action__Test_Goal
 * )) before or use
 * robot_action__action__Test_Goal__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_Goal__init(robot_action__action__Test_Goal * msg);

/// Finalize action/Test message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
void
robot_action__action__Test_Goal__fini(robot_action__action__Test_Goal * msg);

/// Create action/Test message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * robot_action__action__Test_Goal__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
robot_action__action__Test_Goal *
robot_action__action__Test_Goal__create();

/// Destroy action/Test message.
/**
 * It calls
 * robot_action__action__Test_Goal__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
void
robot_action__action__Test_Goal__destroy(robot_action__action__Test_Goal * msg);

/// Check for action/Test message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_Goal__are_equal(const robot_action__action__Test_Goal * lhs, const robot_action__action__Test_Goal * rhs);

/// Copy a action/Test message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_Goal__copy(
  const robot_action__action__Test_Goal * input,
  robot_action__action__Test_Goal * output);

/// Initialize array of action/Test messages.
/**
 * It allocates the memory for the number of elements and calls
 * robot_action__action__Test_Goal__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_Goal__Sequence__init(robot_action__action__Test_Goal__Sequence * array, size_t size);

/// Finalize array of action/Test messages.
/**
 * It calls
 * robot_action__action__Test_Goal__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
void
robot_action__action__Test_Goal__Sequence__fini(robot_action__action__Test_Goal__Sequence * array);

/// Create array of action/Test messages.
/**
 * It allocates the memory for the array and calls
 * robot_action__action__Test_Goal__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
robot_action__action__Test_Goal__Sequence *
robot_action__action__Test_Goal__Sequence__create(size_t size);

/// Destroy array of action/Test messages.
/**
 * It calls
 * robot_action__action__Test_Goal__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
void
robot_action__action__Test_Goal__Sequence__destroy(robot_action__action__Test_Goal__Sequence * array);

/// Check for action/Test message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_Goal__Sequence__are_equal(const robot_action__action__Test_Goal__Sequence * lhs, const robot_action__action__Test_Goal__Sequence * rhs);

/// Copy an array of action/Test messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_Goal__Sequence__copy(
  const robot_action__action__Test_Goal__Sequence * input,
  robot_action__action__Test_Goal__Sequence * output);

/// Initialize action/Test message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * robot_action__action__Test_Result
 * )) before or use
 * robot_action__action__Test_Result__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_Result__init(robot_action__action__Test_Result * msg);

/// Finalize action/Test message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
void
robot_action__action__Test_Result__fini(robot_action__action__Test_Result * msg);

/// Create action/Test message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * robot_action__action__Test_Result__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
robot_action__action__Test_Result *
robot_action__action__Test_Result__create();

/// Destroy action/Test message.
/**
 * It calls
 * robot_action__action__Test_Result__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
void
robot_action__action__Test_Result__destroy(robot_action__action__Test_Result * msg);

/// Check for action/Test message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_Result__are_equal(const robot_action__action__Test_Result * lhs, const robot_action__action__Test_Result * rhs);

/// Copy a action/Test message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_Result__copy(
  const robot_action__action__Test_Result * input,
  robot_action__action__Test_Result * output);

/// Initialize array of action/Test messages.
/**
 * It allocates the memory for the number of elements and calls
 * robot_action__action__Test_Result__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_Result__Sequence__init(robot_action__action__Test_Result__Sequence * array, size_t size);

/// Finalize array of action/Test messages.
/**
 * It calls
 * robot_action__action__Test_Result__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
void
robot_action__action__Test_Result__Sequence__fini(robot_action__action__Test_Result__Sequence * array);

/// Create array of action/Test messages.
/**
 * It allocates the memory for the array and calls
 * robot_action__action__Test_Result__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
robot_action__action__Test_Result__Sequence *
robot_action__action__Test_Result__Sequence__create(size_t size);

/// Destroy array of action/Test messages.
/**
 * It calls
 * robot_action__action__Test_Result__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
void
robot_action__action__Test_Result__Sequence__destroy(robot_action__action__Test_Result__Sequence * array);

/// Check for action/Test message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_Result__Sequence__are_equal(const robot_action__action__Test_Result__Sequence * lhs, const robot_action__action__Test_Result__Sequence * rhs);

/// Copy an array of action/Test messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_Result__Sequence__copy(
  const robot_action__action__Test_Result__Sequence * input,
  robot_action__action__Test_Result__Sequence * output);

/// Initialize action/Test message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * robot_action__action__Test_Feedback
 * )) before or use
 * robot_action__action__Test_Feedback__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_Feedback__init(robot_action__action__Test_Feedback * msg);

/// Finalize action/Test message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
void
robot_action__action__Test_Feedback__fini(robot_action__action__Test_Feedback * msg);

/// Create action/Test message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * robot_action__action__Test_Feedback__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
robot_action__action__Test_Feedback *
robot_action__action__Test_Feedback__create();

/// Destroy action/Test message.
/**
 * It calls
 * robot_action__action__Test_Feedback__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
void
robot_action__action__Test_Feedback__destroy(robot_action__action__Test_Feedback * msg);

/// Check for action/Test message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_Feedback__are_equal(const robot_action__action__Test_Feedback * lhs, const robot_action__action__Test_Feedback * rhs);

/// Copy a action/Test message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_Feedback__copy(
  const robot_action__action__Test_Feedback * input,
  robot_action__action__Test_Feedback * output);

/// Initialize array of action/Test messages.
/**
 * It allocates the memory for the number of elements and calls
 * robot_action__action__Test_Feedback__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_Feedback__Sequence__init(robot_action__action__Test_Feedback__Sequence * array, size_t size);

/// Finalize array of action/Test messages.
/**
 * It calls
 * robot_action__action__Test_Feedback__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
void
robot_action__action__Test_Feedback__Sequence__fini(robot_action__action__Test_Feedback__Sequence * array);

/// Create array of action/Test messages.
/**
 * It allocates the memory for the array and calls
 * robot_action__action__Test_Feedback__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
robot_action__action__Test_Feedback__Sequence *
robot_action__action__Test_Feedback__Sequence__create(size_t size);

/// Destroy array of action/Test messages.
/**
 * It calls
 * robot_action__action__Test_Feedback__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
void
robot_action__action__Test_Feedback__Sequence__destroy(robot_action__action__Test_Feedback__Sequence * array);

/// Check for action/Test message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_Feedback__Sequence__are_equal(const robot_action__action__Test_Feedback__Sequence * lhs, const robot_action__action__Test_Feedback__Sequence * rhs);

/// Copy an array of action/Test messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_Feedback__Sequence__copy(
  const robot_action__action__Test_Feedback__Sequence * input,
  robot_action__action__Test_Feedback__Sequence * output);

/// Initialize action/Test message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * robot_action__action__Test_SendGoal_Request
 * )) before or use
 * robot_action__action__Test_SendGoal_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_SendGoal_Request__init(robot_action__action__Test_SendGoal_Request * msg);

/// Finalize action/Test message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
void
robot_action__action__Test_SendGoal_Request__fini(robot_action__action__Test_SendGoal_Request * msg);

/// Create action/Test message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * robot_action__action__Test_SendGoal_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
robot_action__action__Test_SendGoal_Request *
robot_action__action__Test_SendGoal_Request__create();

/// Destroy action/Test message.
/**
 * It calls
 * robot_action__action__Test_SendGoal_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
void
robot_action__action__Test_SendGoal_Request__destroy(robot_action__action__Test_SendGoal_Request * msg);

/// Check for action/Test message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_SendGoal_Request__are_equal(const robot_action__action__Test_SendGoal_Request * lhs, const robot_action__action__Test_SendGoal_Request * rhs);

/// Copy a action/Test message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_SendGoal_Request__copy(
  const robot_action__action__Test_SendGoal_Request * input,
  robot_action__action__Test_SendGoal_Request * output);

/// Initialize array of action/Test messages.
/**
 * It allocates the memory for the number of elements and calls
 * robot_action__action__Test_SendGoal_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_SendGoal_Request__Sequence__init(robot_action__action__Test_SendGoal_Request__Sequence * array, size_t size);

/// Finalize array of action/Test messages.
/**
 * It calls
 * robot_action__action__Test_SendGoal_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
void
robot_action__action__Test_SendGoal_Request__Sequence__fini(robot_action__action__Test_SendGoal_Request__Sequence * array);

/// Create array of action/Test messages.
/**
 * It allocates the memory for the array and calls
 * robot_action__action__Test_SendGoal_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
robot_action__action__Test_SendGoal_Request__Sequence *
robot_action__action__Test_SendGoal_Request__Sequence__create(size_t size);

/// Destroy array of action/Test messages.
/**
 * It calls
 * robot_action__action__Test_SendGoal_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
void
robot_action__action__Test_SendGoal_Request__Sequence__destroy(robot_action__action__Test_SendGoal_Request__Sequence * array);

/// Check for action/Test message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_SendGoal_Request__Sequence__are_equal(const robot_action__action__Test_SendGoal_Request__Sequence * lhs, const robot_action__action__Test_SendGoal_Request__Sequence * rhs);

/// Copy an array of action/Test messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_SendGoal_Request__Sequence__copy(
  const robot_action__action__Test_SendGoal_Request__Sequence * input,
  robot_action__action__Test_SendGoal_Request__Sequence * output);

/// Initialize action/Test message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * robot_action__action__Test_SendGoal_Response
 * )) before or use
 * robot_action__action__Test_SendGoal_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_SendGoal_Response__init(robot_action__action__Test_SendGoal_Response * msg);

/// Finalize action/Test message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
void
robot_action__action__Test_SendGoal_Response__fini(robot_action__action__Test_SendGoal_Response * msg);

/// Create action/Test message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * robot_action__action__Test_SendGoal_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
robot_action__action__Test_SendGoal_Response *
robot_action__action__Test_SendGoal_Response__create();

/// Destroy action/Test message.
/**
 * It calls
 * robot_action__action__Test_SendGoal_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
void
robot_action__action__Test_SendGoal_Response__destroy(robot_action__action__Test_SendGoal_Response * msg);

/// Check for action/Test message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_SendGoal_Response__are_equal(const robot_action__action__Test_SendGoal_Response * lhs, const robot_action__action__Test_SendGoal_Response * rhs);

/// Copy a action/Test message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_SendGoal_Response__copy(
  const robot_action__action__Test_SendGoal_Response * input,
  robot_action__action__Test_SendGoal_Response * output);

/// Initialize array of action/Test messages.
/**
 * It allocates the memory for the number of elements and calls
 * robot_action__action__Test_SendGoal_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_SendGoal_Response__Sequence__init(robot_action__action__Test_SendGoal_Response__Sequence * array, size_t size);

/// Finalize array of action/Test messages.
/**
 * It calls
 * robot_action__action__Test_SendGoal_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
void
robot_action__action__Test_SendGoal_Response__Sequence__fini(robot_action__action__Test_SendGoal_Response__Sequence * array);

/// Create array of action/Test messages.
/**
 * It allocates the memory for the array and calls
 * robot_action__action__Test_SendGoal_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
robot_action__action__Test_SendGoal_Response__Sequence *
robot_action__action__Test_SendGoal_Response__Sequence__create(size_t size);

/// Destroy array of action/Test messages.
/**
 * It calls
 * robot_action__action__Test_SendGoal_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
void
robot_action__action__Test_SendGoal_Response__Sequence__destroy(robot_action__action__Test_SendGoal_Response__Sequence * array);

/// Check for action/Test message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_SendGoal_Response__Sequence__are_equal(const robot_action__action__Test_SendGoal_Response__Sequence * lhs, const robot_action__action__Test_SendGoal_Response__Sequence * rhs);

/// Copy an array of action/Test messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_SendGoal_Response__Sequence__copy(
  const robot_action__action__Test_SendGoal_Response__Sequence * input,
  robot_action__action__Test_SendGoal_Response__Sequence * output);

/// Initialize action/Test message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * robot_action__action__Test_GetResult_Request
 * )) before or use
 * robot_action__action__Test_GetResult_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_GetResult_Request__init(robot_action__action__Test_GetResult_Request * msg);

/// Finalize action/Test message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
void
robot_action__action__Test_GetResult_Request__fini(robot_action__action__Test_GetResult_Request * msg);

/// Create action/Test message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * robot_action__action__Test_GetResult_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
robot_action__action__Test_GetResult_Request *
robot_action__action__Test_GetResult_Request__create();

/// Destroy action/Test message.
/**
 * It calls
 * robot_action__action__Test_GetResult_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
void
robot_action__action__Test_GetResult_Request__destroy(robot_action__action__Test_GetResult_Request * msg);

/// Check for action/Test message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_GetResult_Request__are_equal(const robot_action__action__Test_GetResult_Request * lhs, const robot_action__action__Test_GetResult_Request * rhs);

/// Copy a action/Test message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_GetResult_Request__copy(
  const robot_action__action__Test_GetResult_Request * input,
  robot_action__action__Test_GetResult_Request * output);

/// Initialize array of action/Test messages.
/**
 * It allocates the memory for the number of elements and calls
 * robot_action__action__Test_GetResult_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_GetResult_Request__Sequence__init(robot_action__action__Test_GetResult_Request__Sequence * array, size_t size);

/// Finalize array of action/Test messages.
/**
 * It calls
 * robot_action__action__Test_GetResult_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
void
robot_action__action__Test_GetResult_Request__Sequence__fini(robot_action__action__Test_GetResult_Request__Sequence * array);

/// Create array of action/Test messages.
/**
 * It allocates the memory for the array and calls
 * robot_action__action__Test_GetResult_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
robot_action__action__Test_GetResult_Request__Sequence *
robot_action__action__Test_GetResult_Request__Sequence__create(size_t size);

/// Destroy array of action/Test messages.
/**
 * It calls
 * robot_action__action__Test_GetResult_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
void
robot_action__action__Test_GetResult_Request__Sequence__destroy(robot_action__action__Test_GetResult_Request__Sequence * array);

/// Check for action/Test message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_GetResult_Request__Sequence__are_equal(const robot_action__action__Test_GetResult_Request__Sequence * lhs, const robot_action__action__Test_GetResult_Request__Sequence * rhs);

/// Copy an array of action/Test messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_GetResult_Request__Sequence__copy(
  const robot_action__action__Test_GetResult_Request__Sequence * input,
  robot_action__action__Test_GetResult_Request__Sequence * output);

/// Initialize action/Test message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * robot_action__action__Test_GetResult_Response
 * )) before or use
 * robot_action__action__Test_GetResult_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_GetResult_Response__init(robot_action__action__Test_GetResult_Response * msg);

/// Finalize action/Test message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
void
robot_action__action__Test_GetResult_Response__fini(robot_action__action__Test_GetResult_Response * msg);

/// Create action/Test message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * robot_action__action__Test_GetResult_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
robot_action__action__Test_GetResult_Response *
robot_action__action__Test_GetResult_Response__create();

/// Destroy action/Test message.
/**
 * It calls
 * robot_action__action__Test_GetResult_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
void
robot_action__action__Test_GetResult_Response__destroy(robot_action__action__Test_GetResult_Response * msg);

/// Check for action/Test message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_GetResult_Response__are_equal(const robot_action__action__Test_GetResult_Response * lhs, const robot_action__action__Test_GetResult_Response * rhs);

/// Copy a action/Test message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_GetResult_Response__copy(
  const robot_action__action__Test_GetResult_Response * input,
  robot_action__action__Test_GetResult_Response * output);

/// Initialize array of action/Test messages.
/**
 * It allocates the memory for the number of elements and calls
 * robot_action__action__Test_GetResult_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_GetResult_Response__Sequence__init(robot_action__action__Test_GetResult_Response__Sequence * array, size_t size);

/// Finalize array of action/Test messages.
/**
 * It calls
 * robot_action__action__Test_GetResult_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
void
robot_action__action__Test_GetResult_Response__Sequence__fini(robot_action__action__Test_GetResult_Response__Sequence * array);

/// Create array of action/Test messages.
/**
 * It allocates the memory for the array and calls
 * robot_action__action__Test_GetResult_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
robot_action__action__Test_GetResult_Response__Sequence *
robot_action__action__Test_GetResult_Response__Sequence__create(size_t size);

/// Destroy array of action/Test messages.
/**
 * It calls
 * robot_action__action__Test_GetResult_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
void
robot_action__action__Test_GetResult_Response__Sequence__destroy(robot_action__action__Test_GetResult_Response__Sequence * array);

/// Check for action/Test message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_GetResult_Response__Sequence__are_equal(const robot_action__action__Test_GetResult_Response__Sequence * lhs, const robot_action__action__Test_GetResult_Response__Sequence * rhs);

/// Copy an array of action/Test messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_GetResult_Response__Sequence__copy(
  const robot_action__action__Test_GetResult_Response__Sequence * input,
  robot_action__action__Test_GetResult_Response__Sequence * output);

/// Initialize action/Test message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * robot_action__action__Test_FeedbackMessage
 * )) before or use
 * robot_action__action__Test_FeedbackMessage__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_FeedbackMessage__init(robot_action__action__Test_FeedbackMessage * msg);

/// Finalize action/Test message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
void
robot_action__action__Test_FeedbackMessage__fini(robot_action__action__Test_FeedbackMessage * msg);

/// Create action/Test message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * robot_action__action__Test_FeedbackMessage__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
robot_action__action__Test_FeedbackMessage *
robot_action__action__Test_FeedbackMessage__create();

/// Destroy action/Test message.
/**
 * It calls
 * robot_action__action__Test_FeedbackMessage__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
void
robot_action__action__Test_FeedbackMessage__destroy(robot_action__action__Test_FeedbackMessage * msg);

/// Check for action/Test message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_FeedbackMessage__are_equal(const robot_action__action__Test_FeedbackMessage * lhs, const robot_action__action__Test_FeedbackMessage * rhs);

/// Copy a action/Test message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_FeedbackMessage__copy(
  const robot_action__action__Test_FeedbackMessage * input,
  robot_action__action__Test_FeedbackMessage * output);

/// Initialize array of action/Test messages.
/**
 * It allocates the memory for the number of elements and calls
 * robot_action__action__Test_FeedbackMessage__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_FeedbackMessage__Sequence__init(robot_action__action__Test_FeedbackMessage__Sequence * array, size_t size);

/// Finalize array of action/Test messages.
/**
 * It calls
 * robot_action__action__Test_FeedbackMessage__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
void
robot_action__action__Test_FeedbackMessage__Sequence__fini(robot_action__action__Test_FeedbackMessage__Sequence * array);

/// Create array of action/Test messages.
/**
 * It allocates the memory for the array and calls
 * robot_action__action__Test_FeedbackMessage__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
robot_action__action__Test_FeedbackMessage__Sequence *
robot_action__action__Test_FeedbackMessage__Sequence__create(size_t size);

/// Destroy array of action/Test messages.
/**
 * It calls
 * robot_action__action__Test_FeedbackMessage__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
void
robot_action__action__Test_FeedbackMessage__Sequence__destroy(robot_action__action__Test_FeedbackMessage__Sequence * array);

/// Check for action/Test message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_FeedbackMessage__Sequence__are_equal(const robot_action__action__Test_FeedbackMessage__Sequence * lhs, const robot_action__action__Test_FeedbackMessage__Sequence * rhs);

/// Copy an array of action/Test messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_action
bool
robot_action__action__Test_FeedbackMessage__Sequence__copy(
  const robot_action__action__Test_FeedbackMessage__Sequence * input,
  robot_action__action__Test_FeedbackMessage__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_ACTION__ACTION__DETAIL__TEST__FUNCTIONS_H_
