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

/* Development of this module has been funded by the Monterey Bay Aquarium
Research Institute (MBARI) and the David and Lucile Packard Foundation */

#include "helper/LrauvCommsFixture.hh"
#include "lrauv_range_bearing_request.pb.h"
#include "lrauv_range_bearing_response.pb.h"

using namespace lrauv_ignition_plugins;

class RangeBearingClient
{

public: RangeBearingClient(std::string address) :
  request_counter(2)
{
  this->pub = this->node.Advertise<msgs::LRAUVRangeBearingRequest>(
    "/"+address+"/range_bearing/requests");
  this->node.Subscribe("/"+address+"/range_bearing/responses",
    &RangeBearingClient::onCallback,
    this
  );
}

public: msgs::LRAUVRangeBearingResponse RequestRange(uint32_t address)
{
  msgs::LRAUVRangeBearingRequest req;
  req.set_to(address);
  req.set_req_id(this->request_counter);

  this->recieved = false;

  this->pub.Publish(req);

  igndbg << "Published request" << "\n";

  std::unique_lock<std::mutex> lk(this->mtx);
  cv.wait(lk, [this]{return this->recieved;});
  //sleep(2.0);
  this->request_counter++;

  return this->lastResponse;
}

public: void onCallback(const msgs::LRAUVRangeBearingResponse& resp)
{
  igndbg << "Recieved message\n";
  if(resp.req_id() == this->request_counter)
  {
    {
      std::lock_guard<std::mutex> lk(this->mtx);
      this->lastResponse = resp;
      this->recieved = true;
    }
    cv.notify_all();
  }
}

private: ignition::transport::Node node;

private: ignition::transport::Node::Publisher pub;

private: uint32_t request_counter;

private: std::mutex mtx;

private: std::condition_variable cv;

private: msgs::LRAUVRangeBearingResponse lastResponse;

private: bool recieved;
};

TEST_F(LrauvCommsFixture, TestRangingAccuracy)
{
  /// Needs to have the server call configure on the plugin before requesting.
  this->fixture->Server()->Run(true, 100, false);
  this->fixture->Server()->Run(false, 2000, false);
  
  RangeBearingClient client("box1");
  auto result = client.RequestRange(2);

  EXPECT_EQ(result.req_id(), 2);
  EXPECT_NEAR(result.range(), 20, 5);
  EXPECT_NEAR(result.bearing().x(), 20, 1e-3);
  EXPECT_NEAR(result.bearing().y(), 1.57, 1e-2);
  EXPECT_NEAR(result.bearing().z(), 1.57, 1e-2);
}
