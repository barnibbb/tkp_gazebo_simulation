#include <fstream>
#include <iostream>

#include "uav_controller.h"
#include "data_recorder.h"

int main (int argc, char** argv)
{
    ros::init(argc, argv, "gazebo_uav_simulation");
    
    const std::string package_path = "/home/barni/tkp_ws/src/tkp_gazebo_simulation/";
    const std::string control_parameters_file = package_path + "files/parameters/uav_control_parameters.txt";
    const std::string record_parameters_file = package_path + "files/parameters/uav_record_parameters.txt";
    const std::string waypoints_file = package_path + "files/routes/uav_trajectory_90.txt";

    tkp::simulation::Parameters control_parameters = tkp::simulation::readParameters(control_parameters_file);
    tkp::simulation::Parameters record_parameters = tkp::simulation::readParameters(record_parameters_file);

    tkp::simulation::GazeboUAVController uav_controller(control_parameters);

    uav_controller.readWaypoints(waypoints_file);

    tkp::simulation::DataRecorder data_recorder(record_parameters);

    ros::NodeHandle node_handle;

    ros::Publisher velocity_publisher = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::Rate rate(10);

    while (!uav_controller.isRouteCompleted())
    {
        geometry_msgs::Pose pose = data_recorder.getOdometry().pose.pose;

        geometry_msgs::Point position = pose.position;
        geometry_msgs::Quaternion quaternion = pose.orientation;

        double roll, pitch, yaw;

        tf::Quaternion tf_quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
        tf::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);

        tkp::simulation::State current_state { position.x, position.y, position.z, roll, pitch, yaw };

        geometry_msgs::Twist velocity_message = uav_controller.computeVelocity(current_state);

        if (uav_controller.isRouteCompleted())
        {
            velocity_message.linear.x = 0;
            velocity_message.linear.y = 0;
        }

        velocity_publisher.publish(velocity_message);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
