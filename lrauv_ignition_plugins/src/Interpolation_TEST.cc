/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

/*
 * Development of this module has been funded by the Monterey Bay Aquarium
 * Research Institute (MBARI) and the David and Lucile Packard Foundation
 */

#include <gtest/gtest.h>

#include "Interpolation.hh"

namespace tethys
{

TEST(InterpolationTest, SetAndGetMethod)
{
  // Constructor using default arg
  Interpolation interp_dft;
  EXPECT_EQ(interp_dft.Method(), TRILINEAR);

  // Constructor using default arg explicitly
  Interpolation interp1(TRILINEAR);
  EXPECT_EQ(interp1.Method(), TRILINEAR);

  interp1.SetMethod(BARYCENTRIC);
  EXPECT_EQ(interp1.Method(), BARYCENTRIC);

  // Constructor using alternative arg
  Interpolation interp2(BARYCENTRIC);
  EXPECT_EQ(interp2.Method(), BARYCENTRIC);

  interp2.SetMethod(TRILINEAR);
  EXPECT_EQ(interp2.Method(), TRILINEAR);
}

}
