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

  // Negative moves toward nose of vehicle; positive moves toward tail
  double dist = -0.001;

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  lrauv_ignition_plugins::msgs::LRAUVCommand batteryMsg;
  batteryMsg.set_masspositionaction_(dist);
  commandPub.Publish(batteryMsg);
  std::cout << "Commanding mass shifter to " << dist << std::endl;

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  lrauv_ignition_plugins::msgs::LRAUVCommand batteryReverseMsg;
  batteryReverseMsg.set_masspositionaction_(-dist);
  commandPub.Publish(batteryReverseMsg);
  std::cout << "Commanding mass shifter to " << -dist << std::endl;
}
