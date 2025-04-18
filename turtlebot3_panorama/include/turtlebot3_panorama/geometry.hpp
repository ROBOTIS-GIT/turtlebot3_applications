// Copyright 2012 Yujin Robot.
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
//
// Description:
//   This header defines basic angle utility functions for the
//   turtlebot3_panorama module. It includes functions to convert between
//   degrees and radians, and to normalize angles to the [-π, π] range.
//   These utilities are used for rotational computations in the panorama algorithm.
//
// Authors: Younghun Ju, Jihoon Lee, Marcus Liebhardt, YeonSoo Noh

#ifndef TURTLEBOT3_PANORAMA__GEOMETRY_HPP_
#define TURTLEBOT3_PANORAMA__GEOMETRY_HPP_

#include <cmath>


namespace turtlebot3_panorama
{

template<typename T>
T degrees_to_radians(const T & degrees)
{
  constexpr double degs_to_rads = M_PI / 180.0;
  return degrees * degs_to_rads;
}

template<typename T>
T radians_to_degrees(const T & radians)
{
  constexpr double rads_to_degs = 180.0 / M_PI;
  return radians * rads_to_degs;
}

}  // namespace turtlebot3_panorama
#endif  // TURTLEBOT3_PANORAMA__GEOMETRY_HPP_
