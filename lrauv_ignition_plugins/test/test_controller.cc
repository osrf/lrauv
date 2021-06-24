/**
 * This is a stupidly simple test controller
 * that wiggles the fins a bit and then commands the robot
 * to charge forward.
 */
#include <chrono>
#include <thread>

#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include "lrauv_command.pb.h"

int main(int argc, char** argv)
{
  ignition::transport::Node node;
  auto commandTopic = "/tethys/command_topic";
  auto commandPub =
    node.Advertise<lrauv_ignition_plugins::msgs::LRAUVCommand>(commandTopic);

  double angle = 0.17;

  // Wiggle rudder
  for (int i = 0; i < 10; i++)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    lrauv_ignition_plugins::msgs::LRAUVCommand rudderMsg;
    angle *= -1;
    rudderMsg.set_buoyancyaction_(300); // Keep it stable
    rudderMsg.set_rudderangleaction_(angle);
    commandPub.Publish(rudderMsg);
    std::cout << "moving rudder to " << angle << "\n";
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  lrauv_ignition_plugins::msgs::LRAUVCommand rudderStopMsg;
  rudderStopMsg.set_buoyancyaction_(300);
  rudderStopMsg.set_rudderangleaction_(0);
  commandPub.Publish(rudderStopMsg);
  std::cout << "moving rudder to " << angle << "\n";

  // Wiggle elevator
  for (int i = 0; i < 10; i++)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    lrauv_ignition_plugins::msgs::LRAUVCommand elevatorMsg;
    angle *= -1;
    elevatorMsg.set_buoyancyaction_(300);
    elevatorMsg.set_elevatorangleaction_(angle);
    commandPub.Publish(elevatorMsg);
    std::cout << "moving elevator to " << angle << "\n";
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  lrauv_ignition_plugins::msgs::LRAUVCommand elevatorStopMsg;
  elevatorStopMsg.set_buoyancyaction_(300);
  elevatorStopMsg.set_elevatorangleaction_(0);
  commandPub.Publish(elevatorStopMsg);
  std::cout << "moving elevator to " << angle << "\n";

  // Charge forward
  lrauv_ignition_plugins::msgs::LRAUVCommand thrustMsg;
  thrustMsg.set_buoyancyaction_(300);
  thrustMsg.set_propomegaaction_(30);
  commandPub.Publish(thrustMsg);
  std::cout << "charging forward!\n";

  std::this_thread::sleep_for(std::chrono::milliseconds(10000));
  lrauv_ignition_plugins::msgs::LRAUVCommand stopMsg;
  stopMsg.set_buoyancyaction_(300);
  stopMsg.set_propomegaaction_(0);
  commandPub.Publish(stopMsg);
  std::cout << "stop\n";
}
