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

#ifndef LRAUVIGNITIONPLUGINS_LRAUVTESTFIXTURE_HH_
#define LRAUVIGNITIONPLUGINS_LRAUVTESTFIXTURE_HH_

#include <gtest/gtest.h>

#include <ignition/gazebo/TestFixture.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/World.hh>
#include <ignition/math/Angle.hh>
#include <ignition/transport/Node.hh>

#include "lrauv_command.pb.h"

#include "TestConstants.hh"

using namespace std::chrono_literals;

/// \brief Helper function to get the absolute difference between 2 angles.
/// The result is normalized to [-pi, pi].
/// \param[in] _a First angle
double angleDiff(double _a, double _b)
{
  auto diff = ignition::math::Angle(_a - _b);
  diff.Normalize();
  return diff.Radian();
}

/// \brief Convenient fixture that provides boilerplate code common to most
/// LRAUV tests.
class LrauvTestFixture : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    ignition::common::Console::SetVerbosity(4);

    // Setup transport
    auto commandTopic = "/tethys/command_topic";
    this->commandPub =
      this->node.Advertise<lrauv_ignition_plugins::msgs::LRAUVCommand>(
      commandTopic);

    // Setup fixture
    this->fixture = std::make_unique<ignition::gazebo::TestFixture>(
        ignition::common::joinPaths(
        std::string(PROJECT_SOURCE_PATH), "worlds", "buoyant_tethys.sdf"));

    fixture->OnPostUpdate(
      [&](const ignition::gazebo::UpdateInfo &_info,
      const ignition::gazebo::EntityComponentManager &_ecm)
      {
        auto worldEntity = ignition::gazebo::worldEntity(_ecm);
        ignition::gazebo::World world(worldEntity);

        auto modelEntity = world.ModelByName(_ecm, "tethys");
        EXPECT_NE(ignition::gazebo::kNullEntity, modelEntity);

        this->tethysPoses.push_back(
            ignition::gazebo::worldPose(modelEntity, _ecm));
        this->iterations++;
      });
    fixture->Finalize();
  }

  /// \brief Keep publishing a command while a condition is met or it times out
  /// \param[in] _msg Command to be published
  /// \param[in] _continue Function that returns true if we should keep
  /// publishing, that is, something that is initially true but should become
  /// false as the command is processed.
  public: void PublishCommandWhile(
      const lrauv_ignition_plugins::msgs::LRAUVCommand &_msg,
      std::function<bool()> _continue)
  {
    int maxSleep{30};
    int sleep{0};
    for (; sleep < maxSleep && _continue(); ++sleep)
    {
      this->commandPub.Publish(_msg);
      this->fixture->Server()->Run(true, 300, false);
      std::this_thread::sleep_for(100ms);
    }
    EXPECT_LT(sleep, maxSleep);
  }

  /// \brief Check that a pose is within a given range.
  /// \param[in] _index Pose index in tethysPoses
  /// \param[in] _refPose Reference pose
  /// \param[in] _posTols Tolerances for X, Y, Z
  /// \param[in] _rotTols Tolerances for Roll, Pitch, Yaw
  public: void CheckRange(int _index, const ignition::math::Pose3d &_refPose,
      const ignition::math::Vector3d &_posTols,
      const ignition::math::Vector3d &_rotTols)
  {
    EXPECT_LE(_refPose.Pos().X() - _posTols[0], this->tethysPoses[_index].X()) <<
        "Index: " << _index << ", X, reference: " << _refPose.Pos().X();
    EXPECT_GE(_refPose.Pos().X() + _posTols[0], this->tethysPoses[_index].X()) <<
        "Index: " << _index << ", X, reference: " << _refPose.Pos().X();

    EXPECT_LE(_refPose.Pos().Y() - _posTols[1], this->tethysPoses[_index].Y()) <<
        "Index: " << _index << ", Y, reference: " << _refPose.Pos().Y();
    EXPECT_GE(_refPose.Pos().Y() + _posTols[1], this->tethysPoses[_index].Y()) <<
        "Index: " << _index << ", Y, reference: " << _refPose.Pos().Y();

    EXPECT_LE(_refPose.Pos().Z() - _posTols[2], this->tethysPoses[_index].Z()) <<
        "Index: " << _index << ", Z, reference: " << _refPose.Pos().Z();
    EXPECT_GE(_refPose.Pos().Z() + _posTols[2], this->tethysPoses[_index].Z()) <<
        "Index: " << _index << ", Z, reference: " << _refPose.Pos().Z();

    auto diffRoll = abs(angleDiff(_refPose.Rot().Roll(),
        this->tethysPoses[_index].Rot().Roll()));
    EXPECT_LE(diffRoll, _rotTols[0]) <<
        "Index: " << _index << ", Roll, difference: " << diffRoll;

    auto diffPitch = abs(angleDiff(_refPose.Rot().Pitch(),
        this->tethysPoses[_index].Rot().Pitch()));
    EXPECT_LE(diffPitch, _rotTols[1]) <<
        "Index: " << _index << ", Pitch, difference: " << diffPitch;

    auto diffYaw = abs(angleDiff(_refPose.Rot().Yaw(),
        this->tethysPoses[_index].Rot().Yaw()));
    EXPECT_LE(diffYaw, _rotTols[2]) <<
        "Index: " << _index << ", Yaw, difference: " << diffYaw;
  }

  /// \brief Check that a pose is within a given range.
  /// \param[in] _index Pose index in tethysPoses
  /// \param[in] _refPose Reference pose
  /// \param[in] _tols Tolerances for position (X) and rotation (Y)
  public: void CheckRange(int _index, const ignition::math::Pose3d &_refPose,
      const ignition::math::Vector2d &_tols = {0.25, 0.1})
  {
    this->CheckRange(_index, _refPose, {_tols.X(), _tols.X(), _tols.X()},
        {_tols.Y(), _tols.Y(), _tols.Y()});
  };

  /// \brief Get a process' PID based on its name
  /// \param[in] _partialProcessName Part of the process name
  public: static pid_t GetPID(const std::string &_partialProcessName)
  {
    char buf[512];
    std::string cmd{"pgrep -f '" + _partialProcessName + "'"};
    FILE *pipe = popen(cmd.c_str(), "r");

    fgets(buf, 512, pipe);
    pid_t pid = strtoul(buf, NULL, 10);

    pclose(pipe);

    return pid;
  }

  public: static void ExportLogs(const std::string &_target)
  {
    if (getenv("EXPORT_LOGS") == NULL)
    {
      ignmsg << "Skipping export of logs" << "\n";
      return;
    }

    int res = system("sudo /home/developer/lrauv_ws/src/lrauv/lrauv_ignition_plugins/plots/unserialize_for_plotting.sh");
    
    if (res != 0)
    {
      ignerr << "Failed to unserialize plots\n";
      return;
    }
    ignmsg << "Finished unserialize, copying.\n";
    
    // WARNING: THIS CAN LEAD TO ARBITRARY CODE EXECUTION
    auto targetfile = _target + ".csv";
    auto cmd = std::string("sudo cp -r /home/developer/lrauv_ws/src/lrauv/lrauv_ignition_plugins/plots/missions/tmp/tmp.csv /results/") + targetfile;
    res = system(cmd.c_str());
    if (res != 0)
    {
      ignerr << "Failed to copy logs\n";
    }
  }

  /// \brief Spin up a new process and execute the LRAUV controller.
  /// \param[in] _mission Mission to run
  /// \param[out] _running Flag to indicate whether the controller is running.
  public: static void ExecLRAUV(const std::string &_mission,
      const std::string &_shortname,
      std::atomic<bool> &_running)
  {
    std::string cmd = std::string(LRAUV_APP_PATH) + "/bin/LRAUV -x 'run " +
        std::string(LRAUV_APP_PATH) + _mission + " quitAtEnd' 2>&1";

    ignmsg << "Running command [" << cmd << "]" << std::endl;

    FILE *pipe = popen(cmd.c_str(), "r");

    if (!pipe)
    {
      ignerr << "Failed to run command [" << cmd << "]." << std::endl;
      return;
    }

    char buffer[128];
    while (!feof(pipe))
    {
      if (fgets(buffer, 128, pipe) != nullptr)
      {
        igndbg << "CMD OUTPUT: " << buffer << std::endl;

        // FIXME: LRAUV app hangs after quit, so force close it
        // See https://github.com/osrf/lrauv/issues/83
        std::string bufferStr{buffer};
        std::string quit{">quit\n"};
        if (auto found = bufferStr.find(quit) != std::string::npos)
        {
          ignmsg << "Quitting application" << std::endl;
          break;
        }
      }
    }

    for (auto process : {"sh.*bin/LRAUV", "bin/LRAUV"})
    {
      auto pid = GetPID(process);

      if (pid == 0)
      {
        ignerr << "Failed to kill process [" << process << "]" << std::endl;
        continue;
      }

      ignmsg << "Killing process [" << process << "] with pid [" << pid << "]" << std::endl;
      kill(pid, 9);
    }

    pclose(pipe);   
    ExportLogs(_shortname);

    ignmsg << "Completed command [" << cmd << "]" << std::endl;

    _running = false;
  }

  /// \brief How many times has OnPostUpdate been run
  public: unsigned int iterations{0u};

  /// \brief All tethys world poses in order
  public: std::vector<ignition::math::Pose3d> tethysPoses;

  /// \brief Test fixture
  public: std::unique_ptr<ignition::gazebo::TestFixture> fixture{nullptr};

  /// \brief Node for communication
  public: ignition::transport::Node node;

  /// \brief Publishes commands
  public: ignition::transport::Node::Publisher commandPub;
};
#endif
