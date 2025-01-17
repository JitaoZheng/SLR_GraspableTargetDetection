// Copyright (c) 2008, Willow Garage, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the Willow Garage nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// This file is originally from:
// https://github.com/ros-perception/image_pipeline/blob/da750d1/depth_image_proc/include/depth_image_proc/depth_traits.h  // NOLINT

#ifndef DEPTHIMAGE_TO_POINTCLOUD2__DEPTH_TRAITS_HPP_
#define DEPTHIMAGE_TO_POINTCLOUD2__DEPTH_TRAITS_HPP_

#include <stdint.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace depthimage_to_pointcloud2
{

// Encapsulate differences between processing float and uint16_t depths
template<typename T>
struct DepthTraits {};
float nearClippingPlane = 0.5;
float farClippingPlane = 1.5;

template<>
struct DepthTraits<uint16_t>
{
  static inline bool valid(uint16_t depth) {return (depth != 0) & (depth != 65535);} // cut out pure white and pure black areas of depth image
  static inline bool nearFarClipping(uint16_t depth) {return (depth * 0.000076f >= nearClippingPlane) & (depth * 0.000076f <= farClippingPlane);} // cut out near and far clipping planes
  static inline float toMeters(uint16_t depth) {return depth * 0.000076f;}   // originally mm * 0.001f; 0.000078f
  static inline uint16_t fromMeters(float depth) {return (depth * 10000.0f) + 0.5f;}
  static inline void initializeBuffer(std::vector<uint8_t> &) {}  // Do nothing
};

template<>
struct DepthTraits<float>
{
  static inline bool valid(float depth) {return std::isfinite(depth);}
  static inline bool nearFarClipping(float depth) {return std::isfinite(depth);}
  static inline float toMeters(float depth) {return depth;}
  static inline float fromMeters(float depth) {return depth;}

  static inline void initializeBuffer(std::vector<uint8_t> & buffer)
  {
    float * start = reinterpret_cast<float *>(&buffer[0]);
    float * end = reinterpret_cast<float *>(&buffer[0] + buffer.size());
    std::fill(start, end, std::numeric_limits<float>::quiet_NaN());
  }
};

}  // namespace depthimage_to_pointcloud2

#endif  // DEPTHIMAGE_TO_POINTCLOUD2__DEPTH_TRAITS_HPP_
