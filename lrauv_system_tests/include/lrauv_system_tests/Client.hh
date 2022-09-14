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

#ifndef LRAUV_SYSTEM_TESTS__CLIENT_HH
#define LRAUV_SYSTEM_TESTS__CLIENT_HH

#include <future>
#include <mutex>
#include <string>
#include <unordered_map>

#include <gz/common/Console.hh>
#include <gz/transport/Node.hh>

namespace lrauv_system_tests
{

/// Client for Ignition Transport topic-based services.
/// \tparam RequestMessageT Request message type.
/// \tparam ResponseMessageT Response message type.
template<typename RequestMessageT, typename ResponseMessageT>
class Client
{
  /// Constructor.
  /// \param[in] _node Node to be used for topic advertisement and subscription.
  /// \param[in] _ns Namespace for the ``requests`` and ``responses`` topics.
  public: Client(gz::transport::Node &_node, const std::string& _ns)
    : requestsCounter(2)
  {
    const std::string responseTopicName = _ns + "/responses";
    if (!_node.Subscribe(responseTopicName, &Client::OnResponse, this))
    {
      throw std::runtime_error("Failed to subscribe to " + responseTopicName);
    }
    this->requestsPublisher =
        _node.Advertise<RequestMessageT>(_ns + "/requests");
  }

  /// Request a response from the remote server.
  /// \param[in] _request Request message instance.
  /// \returns a future response message, to be awaited.
  public: std::future<ResponseMessageT> Request(RequestMessageT _request)
  {
    uint32_t request_id = this->requestsCounter++;
    _request.set_req_id(request_id);

    std::promise<ResponseMessageT> promise;
    std::future<ResponseMessageT> future = promise.get_future();
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      this->responsePromises[request_id] = std::move(promise);
    }
    this->requestsPublisher.Publish(_request);
    gzdbg << "Published request" << "\n";
    return future;
  }

  private: void OnResponse(const ResponseMessageT& response)
  {
    gzdbg << "Received response\n";
    std::lock_guard<std::mutex> lock(this->mutex);
    auto it = this->responsePromises.find(response.req_id());
    if (it == this->responsePromises.end())
    {
      gzwarn << "Received response with unknown request id: "
              << response.req_id() << std::endl;
      return;
    }
    it->second.set_value(response);
    this->responsePromises.erase(it);
  }

  private: gz::transport::Node node;

  private: gz::transport::Node::Publisher requestsPublisher;

  private: std::unordered_map<
    uint32_t, std::promise<ResponseMessageT>> responsePromises;

  private: uint32_t requestsCounter;

  private: std::mutex mutex;
};

}

#endif  // LRAUV_SYSTEM_TESTS__CLIENT_HH
