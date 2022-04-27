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

#ifndef LRAUV_SYSTEM_TESTS__RANGE_BEARING_CLIENT_HH
#define LRAUV_SYSTEM_TESTS__RANGE_BEARING_CLIENT_HH

#include <future>
#include <mutex>
#include <string>
#include <unordered_map>

#include <lrauv_ignition_plugins/lrauv_range_bearing_request.pb.h>
#include <lrauv_ignition_plugins/lrauv_range_bearing_response.pb.h>

#include "lrauv_system_tests/Client.hh"

namespace lrauv_system_tests
{

/// Client to ease range bearing requests
/// over acoustic communication channels.
class RangeBearingClient : public Client<
  lrauv_ignition_plugins::msgs::LRAUVRangeBearingRequest,
  lrauv_ignition_plugins::msgs::LRAUVRangeBearingResponse>
{
  public: using Base = Client<
    lrauv_ignition_plugins::msgs::LRAUVRangeBearingRequest,
    lrauv_ignition_plugins::msgs::LRAUVRangeBearingResponse>;

  /// Constructor.
  /// \param[in] _node Node to be used for topic advertisement and subscription.
  /// \param[in] _address Range bearing server address (typically, the name of
  /// the vehicle).
  public: RangeBearingClient(
      ignition::transport::Node &_node,
      const std::string& _address)
    : Client(_node, "/" + _address + "/range_bearing")
  {
  }

  /// Request range and bearing from an acoustic comms modem.
  /// \param[in] _address Address to be queried.
  /// \return a future response
  public: auto RequestRange(uint32_t _address)
  {
    lrauv_ignition_plugins::msgs::LRAUVRangeBearingRequest request;
    request.set_to(_address);
    return Base::Request(std::move(request));
  }
};
}

#endif // LRAUV_SYSTEM_TESTS__RANGE_BEARING_CLIENT_HH
