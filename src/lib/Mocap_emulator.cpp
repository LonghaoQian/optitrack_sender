#include "Mocap_emulator.h"

Mocap_emulator::Mocap_emulator(const char* PubTopicName,
                               const char* SubTopicName,
                               ros::NodeHandle& n,
                               unsigned int buffersize)
{
    pubmocap_ = n.advertise<optitrack_broadcast::Mocap>(PubTopicName, buffersize);
    subgazebo_ = n.subscribe(SubTopicName, buffersize, &Mocap_emulator::SubscribeFromGazebo,this);
}
Mocap_emulator::~Mocap_emulator()
{

}
void Mocap_emulator::PublishData()
{
    MessageMocap_.pose.position.x = Drone_state_.pose.pose.position.x;
    MessageMocap_.pose.position.y = Drone_state_.pose.pose.position.y;
    MessageMocap_.pose.position.z = Drone_state_.pose.pose.position.z;
    MessageMocap_.twist.linear.x = Drone_state_.twist.twist.linear.x;
    MessageMocap_.twist.linear.y = Drone_state_.twist.twist.linear.y;
    MessageMocap_.twist.linear.z = Drone_state_.twist.twist.linear.z;

    MessageMocap_.pose.orientation.w = Drone_state_.pose.pose.orientation.w;
    MessageMocap_.pose.orientation.x = Drone_state_.pose.pose.orientation.x;
    MessageMocap_.pose.orientation.y = Drone_state_.pose.pose.orientation.y;
    MessageMocap_.pose.orientation.z = Drone_state_.pose.pose.orientation.z;

    quaternion(0) = MessageMocap_.pose.orientation.w;
    quaternion(1) = MessageMocap_.pose.orientation.x;
    quaternion(2) = MessageMocap_.pose.orientation.y;
    quaternion(3) = MessageMocap_.pose.orientation.z;

    R_IB = QuaterionToRotationMatrix(quaternion);
    // the angular velocity from gazebo is in inertial frame
    omega_i(0) = Drone_state_.twist.twist.angular.x;
    omega_i(1) = Drone_state_.twist.twist.angular.y;
    omega_i(2) = Drone_state_.twist.twist.angular.z;

    omega_b = R_IB.transpose() * omega_i; 
    // publish angular velocity in body-fixed frame
    MessageMocap_.twist.angular.x = omega_b(0);
    MessageMocap_.twist.angular.y = omega_b(1);
    MessageMocap_.twist.angular.z = omega_b(2);

    MessageMocap_.header = Drone_state_.header;
    pubmocap_.publish(MessageMocap_);
}
void Mocap_emulator::SubscribeFromGazebo(const nav_msgs::Odometry& msg)
{
    // waiting for plugin message
    Drone_state_ = msg;
    // publish the plugin message
    PublishData();
}
