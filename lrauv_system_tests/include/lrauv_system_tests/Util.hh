/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#ifndef LRAUV_SYSTEM_TESTS__UTIL_HH
#define LRAUV_SYSTEM_TESTS__UTIL_HH

#include <chrono>
#include <future>
#include <vector>

namespace lrauv_system_tests
{

/// A truthy, boolean-like object with an expiration time,
/// particularly useful to implement timeouts in loops.
class Timeout
{
  /// Construct using a `_timeout` duration
  public: explicit Timeout(const std::chrono::nanoseconds &_timeout)
    : deadline(_timeout + std::chrono::steady_clock::now())
  {
  }

  /// Returns the time till timeout expiration
  public: std::chrono::nanoseconds TimeRemaining() const {
    auto now = std::chrono::steady_clock::now();
    return std::max(this->deadline - now, std::chrono::nanoseconds(0));
  }

  public: explicit operator bool() const {
    return std::chrono::steady_clock::now() > this->deadline;
  }

  private: std::chrono::steady_clock::time_point deadline;
};

}

#endif  // LRAUV_SYSTEM_TESTS__UTIL_HH
