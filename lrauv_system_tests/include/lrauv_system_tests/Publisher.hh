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

#ifndef LRAUV_SYSTEM_TESTS__PUBLISHER_HH
#define LRAUV_SYSTEM_TESTS__PUBLISHER_HH

#include <chrono>

#include <gz/transport/Node.hh>

namespace lrauv_system_tests
{

/// Wait for `_publisher` connections i.e. subscribers.
/// \param[in] _publisher Publisher instance to be checked.
/// \param[in] _timeout Maximum time to wait for connections.
/// \return true if `_publisher` has connections, false otherwise.
bool WaitForConnections(
  const gz::transport::Node::Publisher &_publisher,
  const std::chrono::nanoseconds _timeout);

}

#endif // LRAUV_SYSTEM_TESTS__PUBLISHER_HH
