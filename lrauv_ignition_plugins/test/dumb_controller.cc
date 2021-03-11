/**
 * This is a stupidly simple test controller 
 * that wiggles the fins a bit and then commands the robot
 * to charge forward.
 */ 

#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include "lrauv_command.pb.h"
#include <chrono>
#include <thread>

int main(int argc, char** argv)
{
    ignition::transport::Node node;
    auto commandTopic = "command_topic";
    auto commandPub = 
    node.Advertise<lrauv_ignition_plugins::msgs::LRAUVCommand>(commandTopic);

    double angle = 0.17;
    
    // Wiggle rudder
    for(int i = 0; i < 10; i++)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        lrauv_ignition_plugins::msgs::LRAUVCommand msg;
        angle *= -1;
        msg.set_rudderangle_(angle);
        commandPub.Publish(msg);
        std::cout << "moving rudder to " <<angle <<"\n";
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    lrauv_ignition_plugins::msgs::LRAUVCommand msg2;
    msg2.set_rudderangle_(0);
    commandPub.Publish(msg2);
    std::cout << "moving rudder to " <<angle <<"\n";

    // Wiggle rudder
    for(int i = 0; i < 10; i++)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        lrauv_ignition_plugins::msgs::LRAUVCommand msg;
        angle *= -1;
        msg.set_elevatorangle_(angle);
        commandPub.Publish(msg);
        std::cout << "moving elevator to " <<angle <<"\n";
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    lrauv_ignition_plugins::msgs::LRAUVCommand msg3;
    msg3.set_elevatorangle_(0);
    commandPub.Publish(msg3);
    std::cout << "moving elevator to " <<angle <<"\n";

    //Charge forward
    lrauv_ignition_plugins::msgs::LRAUVCommand thrust_msg;
    thrust_msg.set_propomega_(300);
    commandPub.Publish(thrust_msg);
    std::cout << "charging forward!\n";

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    lrauv_ignition_plugins::msgs::LRAUVCommand thrust_msg2;
    thrust_msg2.set_propomega_(0);
    commandPub.Publish(thrust_msg2);
    std::cout << "stop\n";    

}