#include <ros/ros.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "parameters.h"

namespace tkp
{
    namespace simulation
    {
        class DataRecorder
        {
        public:
            DataRecorder(Parameters& parameters);

            void saveData();

            void readOdometry(const nav_msgs::Odometry& odometry_message);
            void readGNSS(const sensor_msgs::NavSatFix& gnss_message);
            void readImage(const sensor_msgs::ImageConstPtr& image_message);

            nav_msgs::Odometry getOdometry() const;
            sensor_msgs::NavSatFix getGNSS() const;

        private:
            RecorderParameters m_parameters;

            ros::NodeHandle m_node_handle;

            image_transport::ImageTransport m_image_transport;

            ros::Subscriber m_odometry_subscriber;
            ros::Subscriber m_gnss_subscriber;
            ros::Subscriber m_imu_subscriber;

            image_transport::Subscriber m_image_subscriber;

            nav_msgs::Odometry m_current_odometry;
            sensor_msgs::NavSatFix m_current_gnss;

            cv_bridge::CvImagePtr m_current_image = nullptr;
        };

    } // namespace simulation
    
} // namespace tkp
