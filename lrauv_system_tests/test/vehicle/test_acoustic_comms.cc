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

#include <chrono>
#include <condition_variable>
#include <mutex>

#include <lrauv_ignition_plugins/lrauv_acoustic_message.pb.h>

#include <lrauv_ignition_plugins/comms/CommsClient.hh>
#include <lrauv_ignition_plugins/comms/CommsPacket.hh>

#include <google/protobuf/util/message_differencer.h>

#include "lrauv_system_tests/TestFixture.hh"

using namespace tethys;
using namespace lrauv_system_tests;

using MessageDifferencer =
    google::protobuf::util::MessageDifferencer;

TEST(AcousticComms, PacketConversions)
{
  LRAUVAcousticMessage message;
  message.set_to(20);
  message.set_from(30);
  message.set_type(LRAUVAcousticMessageType);
  message.set_data("test");

  const auto now = std::chrono::steady_clock::now();
  const auto vector = gz::math::Vector3d(0, 0, 1);
  const auto packet = CommsPacket::make(message, vector, now);
  const auto encoded = packet.ToInternalMsg();
  const auto packet2 = CommsPacket::make(encoded);
  EXPECT_EQ(packet, packet2);

  const auto decoded = packet.ToExternalMsg();
  EXPECT_TRUE(MessageDifferencer::Equals(decoded, message));
}

TEST(AcousticComms, BasicSendReceive)
{
  TestFixture fixture("acoustic_comms_fixture.sdf");

  constexpr int senderAddress = 2;
  CommsClient sender(senderAddress, [](const auto){});

  bool messageReceived = false;
  std::mutex messageArrivalMutex;
  std::condition_variable messageArrival;
  constexpr int receiverAddress = 3;
  CommsClient receiver(receiverAddress, [&](const auto message)
  {
    ASSERT_EQ(message.data(), "test_message");
    {
      std::lock_guard<std::mutex> lock(messageArrivalMutex);
      messageReceived = true;
    }
    messageArrival.notify_all();
  });

  fixture.Step(1000u);

  LRAUVAcousticMessage message;
  message.set_to(receiverAddress);
  message.set_from(senderAddress);
  message.set_type(LRAUVAcousticMessageType);
  message.set_data("test_message");
  sender.SendPacket(message);

  fixture.Step(1000u);

  using namespace std::literals::chrono_literals;
  std::unique_lock<std::mutex> lock(messageArrivalMutex);
  EXPECT_TRUE(messageArrival.wait_for(
      lock, 5s, [&] { return messageReceived; }));
}
