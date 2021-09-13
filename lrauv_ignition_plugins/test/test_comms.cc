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

#include <lrauv_ignition_plugins/comms/CommsClient.hh>
#include <lrauv_ignition_plugins/comms/CommsPacket.hh>

#include <google/protobuf/util/message_differencer.h>

#include "helper/LrauvCommsFixture.hh"

using namespace tethys;

TEST(CommsBasicUnitTest, CommsPacketConversions)
{
  using AcousticMsg = lrauv_ignition_plugins::msgs::LRAUVAcousticMessage;
  using MessageDifferencer = google::protobuf::util::MessageDifferencer;
  
  AcousticMsg msg;
  msg.set_to(20);
  msg.set_from(30);
  msg.set_type(AcousticMsg::MessageType::LRAUVAcousticMessage_MessageType_Other);
  msg.set_data("test_message");

  auto now = std::chrono::steady_clock::now();
  auto vector = ignition::math::Vector3d(0, 0, 1);
  auto packet = CommsPacket::make(msg, vector, now); 
  auto encoded = packet.ToInternalMsg();
  auto packet2 = CommsPacket::make(encoded);

  ASSERT_EQ(packet, packet2);

  auto decoded = packet.ToExternalMsg();

  ASSERT_TRUE(MessageDifferencer::Equals(decoded, msg));
}

TEST_F(LrauvCommsFixture, TestBasicSendRecieve)
{
  using AcousticMsg = lrauv_ignition_plugins::msgs::LRAUVAcousticMessage;

  CommsClient client1(1, [](const auto message){});

  bool recieved_message = false;

  CommsClient client2(2, [this, &recieved_message](const auto message)
  {
    ASSERT_EQ(message.data(), "test_message");
    std::lock_guard<std::mutex> lock(this->mtx);
    recieved_message = true;
  });
  
  AcousticMsg msg;
  msg.set_to(2);
  msg.set_from(1);
  msg.set_type(
    AcousticMsg::MessageType::LRAUVAcousticMessage_MessageType_Other);
  msg.set_data("test_message");
  this->fixture->Server()->Run(false, 1000, false);
  client1.SendPacket(msg);

  while(!recieved_message)
  {
    sleep(1);
  }

}