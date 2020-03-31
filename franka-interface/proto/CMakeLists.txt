cmake_minimum_required (VERSION 3.4)

project(proto CXX)

set(CMAKE_CXX_STANDARD 14)

find_package(Protobuf REQUIRED)

protobuf_generate_cpp(PROTO_FEEDBACK_CONTROLLER_PARAMS_MSG PROTO_FEEDBACK_CONTROLLER_PARAMS_MSG_HDRS feedback_controller_params_msg.proto)
protobuf_generate_cpp(PROTO_ROBOT_STATE_MSG PROTO_ROBOT_STATE_MSG_HDRS robot_state_msg.proto)
protobuf_generate_cpp(PROTO_SENSOR_MSG PROTO_SENSOR_MSG_HDRS sensor_msg.proto)
protobuf_generate_cpp(PROTO_TERMINATION_HANDLER_PARAMS_MSG PROTO_TERMINATION_HANDLER_PARAMS_MSG_HDRS termination_handler_params_msg.proto)
protobuf_generate_cpp(PROTO_TRAJECTORY_GENERATOR_PARAMS_MSG PROTO_TRAJECTORY_GENERATOR_PARAMS_MSG_HDRS trajectory_generator_params_msg.proto)


set(ProtobufIncludePath ${CMAKE_CURRENT_BINARY_DIR}
        CACHE INTERNAL "Path to generated protobuf files.")

add_library(proto SHARED ${PROTO_FEEDBACK_CONTROLLER_PARAMS_MSG} ${PROTO_FEEDBACK_CONTROLLER_PARAMS_MSG_HDRS}
						 ${PROTO_ROBOT_STATE_MSG} ${PROTO_ROBOT_STATE_MSG_HDRS}
						 ${PROTO_SENSOR_MSG} ${PROTO_SENSOR_MSG_HDRS}
                         ${PROTO_TERMINATION_HANDLER_PARAMS_MSG} ${PROTO_TERMINATION_HANDLER_PARAMS_MSG_HDRS}
                         ${PROTO_TRAJECTORY_GENERATOR_PARAMS_MSG} ${PROTO_TRAJECTORY_GENERATOR_PARAMS_MSG_HDRS})
target_link_libraries(proto ${PROTOBUF_LIBRARIES})
target_include_directories(proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

# set_target_properties(proto PROPERTIES POSITION_INDEPENDENT_CODE ON)
