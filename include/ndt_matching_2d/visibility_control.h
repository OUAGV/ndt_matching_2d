// Copyright (c) 2022 OUXT Polaris
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

#ifndef NDT_MATCHING_2D__VISIBILITY_CONTROL_H_
#define NDT_MATCHING_2D__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the NdtMatching2ds on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define NDT_MATCHING_2D_EXPORT __attribute__((dllexport))
#define NDT_MATCHING_2D_IMPORT __attribute__((dllimport))
#else
#define NDT_MATCHING_2D_EXPORT __declspec(dllexport)
#define NDT_MATCHING_2D_IMPORT __declspec(dllimport)
#endif
#ifdef NDT_MATCHING_2D_BUILDING_LIBRARY
#define NDT_MATCHING_2D_PUBLIC NDT_MATCHING_2D_EXPORT
#else
#define NDT_MATCHING_2D_PUBLIC NDT_MATCHING_2D_IMPORT
#endif
#define NDT_MATCHING_2D_PUBLIC_TYPE NDT_MATCHING_2D_PUBLIC
#define NDT_MATCHING_2D_LOCAL
#else
#define NDT_MATCHING_2D_EXPORT __attribute__((visibility("default")))
#define NDT_MATCHING_2D_IMPORT
#if __GNUC__ >= 4
#define NDT_MATCHING_2D_PUBLIC __attribute__((visibility("default")))
#define NDT_MATCHING_2D_LOCAL __attribute__((visibility("hidden")))
#else
#define NDT_MATCHING_2D_PUBLIC
#define NDT_MATCHING_2D_LOCAL
#endif
#define NDT_MATCHING_2D_PUBLIC_TYPE
#endif

#endif  // NDT_MATCHING_2D__VISIBILITY_CONTROL_H_
