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

#include <cstdio>
#include <string>
#include <thread>

namespace lrauv_system_tests
{

/// Handler for an MBARI's LRAUV controller process.
class LRAUVController
{
 public: ~LRAUVController();

 /// Whether the controller process is still running or not.
 public: bool IsRunning() const;

 /// Kill controller process if running.
 public: void Kill();

 /// Start an LRAUV controller process to run a `_mission`.
 /// \param[in] _mission Path to mission XML file,
 /// relative to MBARI's root application directory.
 /// \return handler instance for underlying controller process.
 public: static LRAUVController Execute(const std::string &_mission);

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
