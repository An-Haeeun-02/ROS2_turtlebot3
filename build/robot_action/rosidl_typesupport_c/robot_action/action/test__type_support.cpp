// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from robot_action:action/Test.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "robot_action/action/detail/test__struct.h"
#include "robot_action/action/detail/test__type_support.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace robot_action
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Test_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Test_Goal_type_support_ids_t;

static const _Test_Goal_type_support_ids_t _Test_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Test_Goal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Test_Goal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Test_Goal_type_support_symbol_names_t _Test_Goal_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_action, action, Test_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_action, action, Test_Goal)),
  }
};

typedef struct _Test_Goal_type_support_data_t
{
  void * data[2];
} _Test_Goal_type_support_data_t;

static _Test_Goal_type_support_data_t _Test_Goal_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Test_Goal_message_typesupport_map = {
  2,
  "robot_action",
  &_Test_Goal_message_typesupport_ids.typesupport_identifier[0],
  &_Test_Goal_message_typesupport_symbol_names.symbol_name[0],
  &_Test_Goal_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Test_Goal_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Test_Goal_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace robot_action

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, robot_action, action, Test_Goal)() {
  return &::robot_action::action::rosidl_typesupport_c::Test_Goal_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "robot_action/action/detail/test__struct.h"
// already included above
// #include "robot_action/action/detail/test__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robot_action
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Test_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Test_Result_type_support_ids_t;

static const _Test_Result_type_support_ids_t _Test_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Test_Result_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Test_Result_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Test_Result_type_support_symbol_names_t _Test_Result_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_action, action, Test_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_action, action, Test_Result)),
  }
};

typedef struct _Test_Result_type_support_data_t
{
  void * data[2];
} _Test_Result_type_support_data_t;

static _Test_Result_type_support_data_t _Test_Result_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Test_Result_message_typesupport_map = {
  2,
  "robot_action",
  &_Test_Result_message_typesupport_ids.typesupport_identifier[0],
  &_Test_Result_message_typesupport_symbol_names.symbol_name[0],
  &_Test_Result_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Test_Result_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Test_Result_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace robot_action

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, robot_action, action, Test_Result)() {
  return &::robot_action::action::rosidl_typesupport_c::Test_Result_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "robot_action/action/detail/test__struct.h"
// already included above
// #include "robot_action/action/detail/test__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robot_action
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Test_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Test_Feedback_type_support_ids_t;

static const _Test_Feedback_type_support_ids_t _Test_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Test_Feedback_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Test_Feedback_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Test_Feedback_type_support_symbol_names_t _Test_Feedback_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_action, action, Test_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_action, action, Test_Feedback)),
  }
};

typedef struct _Test_Feedback_type_support_data_t
{
  void * data[2];
} _Test_Feedback_type_support_data_t;

static _Test_Feedback_type_support_data_t _Test_Feedback_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Test_Feedback_message_typesupport_map = {
  2,
  "robot_action",
  &_Test_Feedback_message_typesupport_ids.typesupport_identifier[0],
  &_Test_Feedback_message_typesupport_symbol_names.symbol_name[0],
  &_Test_Feedback_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Test_Feedback_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Test_Feedback_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace robot_action

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, robot_action, action, Test_Feedback)() {
  return &::robot_action::action::rosidl_typesupport_c::Test_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "robot_action/action/detail/test__struct.h"
// already included above
// #include "robot_action/action/detail/test__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robot_action
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Test_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Test_SendGoal_Request_type_support_ids_t;

static const _Test_SendGoal_Request_type_support_ids_t _Test_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Test_SendGoal_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Test_SendGoal_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Test_SendGoal_Request_type_support_symbol_names_t _Test_SendGoal_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_action, action, Test_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_action, action, Test_SendGoal_Request)),
  }
};

typedef struct _Test_SendGoal_Request_type_support_data_t
{
  void * data[2];
} _Test_SendGoal_Request_type_support_data_t;

static _Test_SendGoal_Request_type_support_data_t _Test_SendGoal_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Test_SendGoal_Request_message_typesupport_map = {
  2,
  "robot_action",
  &_Test_SendGoal_Request_message_typesupport_ids.typesupport_identifier[0],
  &_Test_SendGoal_Request_message_typesupport_symbol_names.symbol_name[0],
  &_Test_SendGoal_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Test_SendGoal_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Test_SendGoal_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace robot_action

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, robot_action, action, Test_SendGoal_Request)() {
  return &::robot_action::action::rosidl_typesupport_c::Test_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "robot_action/action/detail/test__struct.h"
// already included above
// #include "robot_action/action/detail/test__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robot_action
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Test_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Test_SendGoal_Response_type_support_ids_t;

static const _Test_SendGoal_Response_type_support_ids_t _Test_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Test_SendGoal_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Test_SendGoal_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Test_SendGoal_Response_type_support_symbol_names_t _Test_SendGoal_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_action, action, Test_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_action, action, Test_SendGoal_Response)),
  }
};

typedef struct _Test_SendGoal_Response_type_support_data_t
{
  void * data[2];
} _Test_SendGoal_Response_type_support_data_t;

static _Test_SendGoal_Response_type_support_data_t _Test_SendGoal_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Test_SendGoal_Response_message_typesupport_map = {
  2,
  "robot_action",
  &_Test_SendGoal_Response_message_typesupport_ids.typesupport_identifier[0],
  &_Test_SendGoal_Response_message_typesupport_symbol_names.symbol_name[0],
  &_Test_SendGoal_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Test_SendGoal_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Test_SendGoal_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace robot_action

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, robot_action, action, Test_SendGoal_Response)() {
  return &::robot_action::action::rosidl_typesupport_c::Test_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "robot_action/action/detail/test__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robot_action
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Test_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Test_SendGoal_type_support_ids_t;

static const _Test_SendGoal_type_support_ids_t _Test_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Test_SendGoal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Test_SendGoal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Test_SendGoal_type_support_symbol_names_t _Test_SendGoal_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_action, action, Test_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_action, action, Test_SendGoal)),
  }
};

typedef struct _Test_SendGoal_type_support_data_t
{
  void * data[2];
} _Test_SendGoal_type_support_data_t;

static _Test_SendGoal_type_support_data_t _Test_SendGoal_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Test_SendGoal_service_typesupport_map = {
  2,
  "robot_action",
  &_Test_SendGoal_service_typesupport_ids.typesupport_identifier[0],
  &_Test_SendGoal_service_typesupport_symbol_names.symbol_name[0],
  &_Test_SendGoal_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t Test_SendGoal_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Test_SendGoal_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace robot_action

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, robot_action, action, Test_SendGoal)() {
  return &::robot_action::action::rosidl_typesupport_c::Test_SendGoal_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "robot_action/action/detail/test__struct.h"
// already included above
// #include "robot_action/action/detail/test__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robot_action
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Test_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Test_GetResult_Request_type_support_ids_t;

static const _Test_GetResult_Request_type_support_ids_t _Test_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Test_GetResult_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Test_GetResult_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Test_GetResult_Request_type_support_symbol_names_t _Test_GetResult_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_action, action, Test_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_action, action, Test_GetResult_Request)),
  }
};

typedef struct _Test_GetResult_Request_type_support_data_t
{
  void * data[2];
} _Test_GetResult_Request_type_support_data_t;

static _Test_GetResult_Request_type_support_data_t _Test_GetResult_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Test_GetResult_Request_message_typesupport_map = {
  2,
  "robot_action",
  &_Test_GetResult_Request_message_typesupport_ids.typesupport_identifier[0],
  &_Test_GetResult_Request_message_typesupport_symbol_names.symbol_name[0],
  &_Test_GetResult_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Test_GetResult_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Test_GetResult_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace robot_action

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, robot_action, action, Test_GetResult_Request)() {
  return &::robot_action::action::rosidl_typesupport_c::Test_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "robot_action/action/detail/test__struct.h"
// already included above
// #include "robot_action/action/detail/test__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robot_action
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Test_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Test_GetResult_Response_type_support_ids_t;

static const _Test_GetResult_Response_type_support_ids_t _Test_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Test_GetResult_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Test_GetResult_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Test_GetResult_Response_type_support_symbol_names_t _Test_GetResult_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_action, action, Test_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_action, action, Test_GetResult_Response)),
  }
};

typedef struct _Test_GetResult_Response_type_support_data_t
{
  void * data[2];
} _Test_GetResult_Response_type_support_data_t;

static _Test_GetResult_Response_type_support_data_t _Test_GetResult_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Test_GetResult_Response_message_typesupport_map = {
  2,
  "robot_action",
  &_Test_GetResult_Response_message_typesupport_ids.typesupport_identifier[0],
  &_Test_GetResult_Response_message_typesupport_symbol_names.symbol_name[0],
  &_Test_GetResult_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Test_GetResult_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Test_GetResult_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace robot_action

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, robot_action, action, Test_GetResult_Response)() {
  return &::robot_action::action::rosidl_typesupport_c::Test_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "robot_action/action/detail/test__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robot_action
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Test_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Test_GetResult_type_support_ids_t;

static const _Test_GetResult_type_support_ids_t _Test_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Test_GetResult_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Test_GetResult_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Test_GetResult_type_support_symbol_names_t _Test_GetResult_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_action, action, Test_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_action, action, Test_GetResult)),
  }
};

typedef struct _Test_GetResult_type_support_data_t
{
  void * data[2];
} _Test_GetResult_type_support_data_t;

static _Test_GetResult_type_support_data_t _Test_GetResult_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Test_GetResult_service_typesupport_map = {
  2,
  "robot_action",
  &_Test_GetResult_service_typesupport_ids.typesupport_identifier[0],
  &_Test_GetResult_service_typesupport_symbol_names.symbol_name[0],
  &_Test_GetResult_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t Test_GetResult_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Test_GetResult_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace robot_action

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, robot_action, action, Test_GetResult)() {
  return &::robot_action::action::rosidl_typesupport_c::Test_GetResult_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "robot_action/action/detail/test__struct.h"
// already included above
// #include "robot_action/action/detail/test__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robot_action
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Test_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Test_FeedbackMessage_type_support_ids_t;

static const _Test_FeedbackMessage_type_support_ids_t _Test_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Test_FeedbackMessage_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Test_FeedbackMessage_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Test_FeedbackMessage_type_support_symbol_names_t _Test_FeedbackMessage_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_action, action, Test_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_action, action, Test_FeedbackMessage)),
  }
};

typedef struct _Test_FeedbackMessage_type_support_data_t
{
  void * data[2];
} _Test_FeedbackMessage_type_support_data_t;

static _Test_FeedbackMessage_type_support_data_t _Test_FeedbackMessage_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Test_FeedbackMessage_message_typesupport_map = {
  2,
  "robot_action",
  &_Test_FeedbackMessage_message_typesupport_ids.typesupport_identifier[0],
  &_Test_FeedbackMessage_message_typesupport_symbol_names.symbol_name[0],
  &_Test_FeedbackMessage_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Test_FeedbackMessage_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Test_FeedbackMessage_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace robot_action

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, robot_action, action, Test_FeedbackMessage)() {
  return &::robot_action::action::rosidl_typesupport_c::Test_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "action_msgs/msg/goal_status_array.h"
#include "action_msgs/srv/cancel_goal.h"
#include "robot_action/action/test.h"
// already included above
// #include "robot_action/action/detail/test__type_support.h"

static rosidl_action_type_support_t _robot_action__action__Test__typesupport_c;

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_action_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME(
  rosidl_typesupport_c, robot_action, action, Test)()
{
  // Thread-safe by always writing the same values to the static struct
  _robot_action__action__Test__typesupport_c.goal_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, robot_action, action, Test_SendGoal)();
  _robot_action__action__Test__typesupport_c.result_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, robot_action, action, Test_GetResult)();
  _robot_action__action__Test__typesupport_c.cancel_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, action_msgs, srv, CancelGoal)();
  _robot_action__action__Test__typesupport_c.feedback_message_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c, robot_action, action, Test_FeedbackMessage)();
  _robot_action__action__Test__typesupport_c.status_message_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c, action_msgs, msg, GoalStatusArray)();

  return &_robot_action__action__Test__typesupport_c;
}

#ifdef __cplusplus
}
#endif
