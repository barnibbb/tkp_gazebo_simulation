#include "data_recorder.h"

namespace tkp
{
    namespace simulation
    {
        DataRecorder::DataRecorder(Parameters& parameters) : m_image_transport(m_node_handle)
        {
            m_parameters.setParameters(parameters);

            m_odometry_subscriber = m_node_handle.subscribe(m_parameters.odometry_topic, 10, 
                &DataRecorder::readOdometry, dynamic_cast<DataRecorder*>(this));
            m_gnss_subscriber = m_node_handle.subscribe(m_parameters.gnss_topic, 10, 
                &DataRecorder::readGNSS, dynamic_cast<DataRecorder*>(this));
            m_image_subscriber = m_image_transport.subscribe(m_parameters.image_topic, 10, 
                &DataRecorder::readImage, dynamic_cast<DataRecorder*>(this));
        }


        void DataRecorder::readOdometry(const nav_msgs::Odometry& odometry_message)
        {
            m_current_odometry = odometry_message;
        }

        void DataRecorder::readGNSS(const sensor_msgs::NavSatFix& gnss_message)
        {
            m_current_gnss = gnss_message;
        }
        
        void DataRecorder::readImage(const sensor_msgs::ImageConstPtr& image_message)
        {
            try
            {
                m_current_image = cv_bridge::toCvCopy(image_message, sensor_msgs::image_encodings::BGR8);
            }
            catch(const cv_bridge::Exception& error)
            {
                ROS_ERROR("cv_bridge exception: %s", error.what());
            }
        }

        nav_msgs::Odometry DataRecorder::getOdometry() const
        {
            return m_current_odometry;
        }

        sensor_msgs::NavSatFix DataRecorder::getGNSS() const
        {
            return m_current_gnss;
        }

    }
}
