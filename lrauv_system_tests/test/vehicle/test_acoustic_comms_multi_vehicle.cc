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

#include <lrauv_gazebo_plugins/lrauv_acoustic_message.pb.h>

#include <lrauv_gazebo_plugins/comms/CommsClient.hh>
#include <lrauv_gazebo_plugins/comms/CommsPacket.hh>

#include <google/protobuf/util/message_differencer.h>

#include "lrauv_system_tests/TestFixture.hh"

#include "TestConstants.hh"

using namespace tethys;
using namespace lrauv_system_tests;

using MessageDifferencer =
    google::protobuf::util::MessageDifferencer;

TEST(AcousticComms, MultiVehicleTest)
{
  TestFixture fixture(worldPath("acoustic_comms_multi_vehicle.sdf"));

  constexpr int senderAddressTriton = 1;
  CommsClient senderTriton(senderAddressTriton, [](const auto){});

  // Monitor messages on comms for Tethys and Daphne
  std::chrono::time_point<std::chrono::system_clock> receivedTimeTethys;
  bool messageReceivedTethys = false;
  std::mutex messageArrivalMutexTethys;
  std::condition_variable messageArrivalTethys;
  constexpr int receiverAddressTethys = 2;
  CommsClient receiverTethys(receiverAddressTethys, [&](const auto message)
  {
    ASSERT_EQ(message.data(), "test_message");
    {
      std::lock_guard<std::mutex> lock(messageArrivalMutexTethys);
      messageReceivedTethys = true;
      receivedTimeTethys = std::chrono::system_clock::now();
    }
    messageArrivalTethys.notify_all();
  });

  std::chrono::time_point<std::chrono::system_clock> receivedTimeDaphne;
  bool messageReceivedDaphne = false;
  std::mutex messageArrivalMutexDaphne;
  std::condition_variable messageArrivalDaphne;
  constexpr int receiverAddressDaphne = 3;
  CommsClient receiverDaphne(receiverAddressDaphne, [&](const auto message)
  {
    ASSERT_EQ(message.data(), "test_message");
    {
      std::lock_guard<std::mutex> lock(messageArrivalMutexDaphne);
      messageReceivedDaphne = true;
      receivedTimeDaphne = std::chrono::system_clock::now();
    }
    messageArrivalDaphne.notify_all();
  });

  fixture.Step(50u);

  // Send the messages from Triton
  LRAUVAcousticMessage message;
  message.set_to(receiverAddressTethys);
  message.set_from(senderAddressTriton);
  message.set_type(LRAUVAcousticMessageType);
  message.set_data("test_message");
  senderTriton.SendPacket(message);

  message.set_to(receiverAddressDaphne);
  senderTriton.SendPacket(message);

  auto startTime = std::chrono::system_clock::now();

  fixture.Step(100u);

  // Check if the messages were received
  using namespace std::literals::chrono_literals;

  std::unique_lock<std::mutex> lockTethys(messageArrivalMutexTethys);
  EXPECT_TRUE(messageArrivalTethys.wait_for(
      lockTethys, 5s, [&] { return messageReceivedTethys; }));

  std::unique_lock<std::mutex> lockDaphne(messageArrivalMutexDaphne);
  EXPECT_TRUE(messageArrivalDaphne.wait_for(
      lockDaphne, 5s, [&] { return messageReceivedDaphne; }));

  std::chrono::duration<double> diffTethys = receivedTimeTethys - startTime;
  std::chrono::duration<double> diffDaphne = receivedTimeDaphne - startTime;

  // Since the distance between Triton and Daphne is twice the distance between
  // Triton and Tethys, the ratio of time taken by the comms signal to reach them
  // should be in the same ratio.
  EXPECT_NEAR(diffDaphne.count() / diffTethys.count(), 2.0, 0.1);
}
