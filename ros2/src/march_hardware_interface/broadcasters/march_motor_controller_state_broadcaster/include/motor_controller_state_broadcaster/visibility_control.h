// Copyright 2017 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* This header must be included by all rclcpp headers which declare symbols
 * which are defined in the rclcpp library. When not building the rclcpp
 * library, i.e. when using the headers in other package's code, the contents
 * of this header change the visibility of certain symbols which the rclcpp
 * library cannot have, but the consuming code must have inorder to link.
 */

/* This header is copied from: `ros2_controllers/joint_state_broadcaster/include/
 * joint_state_broadcaster/visibility_control.h`. With only the namespaces changed.
 * It is meant to change the export rules for when it is compiled on windows. But have not looked into it that much.
 */

#ifndef MOTOR_CONTROLLER_STATE_BROADCASTER__VISIBILITY_CONTROL_H_
#define MOTOR_CONTROLLER_STATE_BROADCASTER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define MOTOR_CONTROLLER_STATE_BROADCASTER_EXPORT __attribute__((dllexport))
#define MOTOR_CONTROLLER_STATE_BROADCASTER_IMPORT __attribute__((dllimport))
#else
#define MOTOR_CONTROLLER_STATE_BROADCASTER_EXPORT __declspec(dllexport)
#define MOTOR_CONTROLLER_STATE_BROADCASTER_IMPORT __declspec(dllimport)
#endif
#ifdef MOTOR_CONTROLLER_STATE_BROADCASTER_BUILDING_DLL
#define MOTOR_CONTROLLER_STATE_BROADCASTER_PUBLIC MOTOR_CONTROLLER_STATE_BROADCASTER_EXPORT
#else
#define MOTOR_CONTROLLER_STATE_BROADCASTER_PUBLIC MOTOR_CONTROLLER_STATE_BROADCASTER_IMPORT
#endif
#define MOTOR_CONTROLLER_STATE_BROADCASTER_PUBLIC_TYPE MOTOR_CONTROLLER_STATE_BROADCASTER_PUBLIC
#define MOTOR_CONTROLLER_STATE_BROADCASTER_LOCAL
#else
#define MOTOR_CONTROLLER_STATE_BROADCASTER_EXPORT __attribute__((visibility("default")))
#define MOTOR_CONTROLLER_STATE_BROADCASTER_IMPORT
#if __GNUC__ >= 4
#define MOTOR_CONTROLLER_STATE_BROADCASTER_PUBLIC __attribute__((visibility("default")))
#define MOTOR_CONTROLLER_STATE_BROADCASTER_LOCAL __attribute__((visibility("hidden")))
#else
#define MOTOR_CONTROLLER_STATE_BROADCASTER_PUBLIC
#define MOTOR_CONTROLLER_STATE_BROADCASTER_LOCAL
#endif
#define MOTOR_CONTROLLER_STATE_BROADCASTER_PUBLIC_TYPE
#endif

#endif // MOTOR_CONTROLLER_STATE_BROADCASTER__VISIBILITY_CONTROL_H_
