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

#ifndef LRAUV_SYSTEM_TESTS__LRAUVCONTROLLER_HH
#define LRAUV_SYSTEM_TESTS__LRAUVCONTROLLER_HH

#include <signal.h>

#include <cstdio>
#include <string>
#include <thread>
#include <vector>

namespace lrauv_system_tests
{

/// Handler for an MBARI's LRAUV controller process.
class LRAUVController
{
 public: ~LRAUVController();

 /// Whether the controller process is still running or not.
 /// \throws std::system_error if a system call fails.
 public: bool IsRunning();

 /// Kill controller process if running.
 /// \param[in] signal Signal to be sent.
 /// \return true if the signal was delivered, false otherwise.
 /// \throws std::system_error if a system call fails.
 public: bool Kill(int signal);

 /// Wait for controller process to exit.
 /// \return process exit status.
 /// \throws std::system_error if a system call fails.
 public: int Wait();

 /// Start an LRAUV controller process to run `_commands`.
 /// \param[in] _commands A sequence of LRAUV commands to execute.
 /// \return handler instance for underlying controller process.
 /// \throws std::system_error if a system call fails.
 public: static LRAUVController Execute(const std::vector<std::string> &_commands);

 // Constructor.
 // \param[in] pid LRAUV controller process ID.
 // \param[in] _stdout Read end of a pipe connected to
 // the LRAUV controller process standard output.
 private: LRAUVController(int _pid, FILE *_stdout);

 private: std::thread backgroundThread;

 private: int pid;
};

}

#endif // LRAUV_SYSTEM_TESTS__LRAUVCONTROLLER_HH
