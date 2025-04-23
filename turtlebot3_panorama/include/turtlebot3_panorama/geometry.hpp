// Copyright (c) 2013, Yujin Robot
// Licensed under the BSD 3-Clause License
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//   * Neither the name of the Yujin Robot nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 * @file geometry.hpp
 * @brief Simple geometry functions.
 * @date 2012-11-12
 * @author Younghun Ju, Jihoon Lee, Marcus Liebhardt, YeonSoo Noh
 */

#ifndef TURTLEBOT3_PANORAMA__GEOMETRY_HPP_
#define TURTLEBOT3_PANORAMA__GEOMETRY_HPP_

#include <cmath>

namespace turtlebot3_panorama
{

template<typename T>
T degrees_to_radians(const T & degrees)
{
  static const double degs_to_rads = M_PI / 180.0;
  return degrees * degs_to_rads;
}

template<typename T>
T radians_to_degrees(const T & radians)
{
  static const double rads_to_degs = 180.0 / M_PI;
  return radians * rads_to_degs;
}

template<typename T>
T wrap_angle(const T & angle)
{
  float wrapped;
  if ((angle <= M_PI) && (angle >= -M_PI)) {
    wrapped = angle;
  } else if (angle < 0.0) {
    wrapped = fmodf(angle - M_PI, 2.0 * M_PI) + M_PI;
  } else {
    wrapped = fmodf(angle + M_PI, 2.0 * M_PI) - M_PI;
  }
  return wrapped;
}

}  // namespace turtlebot3_panorama
#endif  // TURTLEBOT3_PANORAMA__GEOMETRY_HPP_
