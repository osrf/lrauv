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

/**
 * Keyboard teleop controller
 *
 * Usage:
 *   $ LRAUV_keyboard_teleop <vehicle_name>
 */

#include <unistd.h>
#include <termios.h>

#include <chrono>
#include <iostream>
#include <thread>

#include <ignition/transport.hh>
#include "lrauv_command.pb.h"

char getch()
{
  char buf = 0;
  struct termios old = {0};
  if (tcgetattr(0, &old) < 0)
    perror("tcsetattr()");
  old.c_lflag &= ~ICANON;
  old.c_lflag &= ~ECHO;
  old.c_cc[VMIN] = 1;
  old.c_cc[VTIME] = 0;
  if (tcsetattr(0, TCSANOW, &old) < 0)
    perror("tcsetattr ICANON");
  if (read(0, &buf, 1) < 0)
    perror ("read()");
  old.c_lflag |= ICANON;
  old.c_lflag |= ECHO;
  if (tcsetattr(0, TCSADRAIN, &old) < 0)
    perror ("tcsetattr ~ICANON");
  return (buf);
}

int main(int _argc, char **_argv)
{
  std::string vehicleName("tethys");
  if (_argc > 1)
  {
    vehicleName = _argv[1];
  }
  ignition::transport::Node node;
  auto commandTopic = "/" + vehicleName + "/command_topic";
  auto commandPub =
    node.Advertise<lrauv_ignition_plugins::msgs::LRAUVCommand>(commandTopic);

  double rudder_angle = 0;
  double elevator_angle = 0;
  double thrust = 0;
  while(true)
  {
    std::cout << "\033[2J";
    std::cout << "Keyboard teleop for lrauv" << std::endl;
    std::cout << "Robot namespace set to [" << vehicleName << "]" << std::endl;
    std::cout << "  w  <-- Control elevator to point up (pitch down)" <<std::endl;
    std::cout << "a   d  <-- Control Rudder left/right" << std::endl;
    std::cout << "  s  <-- Point Elevator down (pitch up)" << std::endl;

    std::cout << "Throttle control:" << std::endl;
    std::cout << "\tk - increase thrust " << std::endl;
    std::cout << "\tj - decrease thrust " << std::endl;

    std::cout << "Current state:" << std::endl;
    std::cout << "\tThrust (radians per second): " << thrust << std::endl;
    std::cout << "\tRudder angle (radians): " << rudder_angle << std::endl;
    std::cout << "\tElevator angle (radians): " << elevator_angle << std::endl;

    int res = getch();
    if (res < 0)
      break;

    switch(res)
    {
      case 'k':
        thrust += 0.5;
        break;

      case 'j':
        thrust -= 0.5;
        break;

      case 'w':
        elevator_angle += 0.01;
        break;

      case 's':
        elevator_angle -= 0.01;
        break;

      case 'a':
        rudder_angle -= 0.01;
        break;

      case 'd':
        rudder_angle += 0.01;
        break;
    }

    lrauv_ignition_plugins::msgs::LRAUVCommand cmd;
    cmd.set_propomegaaction_(thrust);
    cmd.set_elevatorangleaction_(elevator_angle);
    cmd.set_rudderangleaction_(rudder_angle);
    cmd.set_buoyancyaction_(0.0005);
    cmd.set_dropweightstate_(1);
    commandPub.Publish(cmd);
  }
}
