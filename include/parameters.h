#pragma once

#include <unordered_map>
#include <string>
#include <fstream>
#include <iostream>

namespace tkp
{
    namespace simulation
    {
        using Parameters = std::unordered_map<std::string, std::string>;

        Parameters readParameters(const std::string parameters_file);

        struct XYControllerParameters
        {
            void setParameters(Parameters& parameters);

            double linear_velocity_big = 3.0;
            double linear_velocity = 0.5;
            double x_tolerance = 0.02;
            double y_tolerance = 0.02;;
        };

        struct XYawControllerParameters
        {
            void setParameters(Parameters& parameters);

            double angular_p = 0.5;
            double linear_velocity = 0.2;
            double distance_tolerance = 0.3;
            double angle_tolerance = 0.1;
        };

        struct RecorderParameters
        {
            void setParameters(Parameters& parameters);

            std::string odometry_topic;
            std::string gnss_topic;
            std::string imu_topic;
            std::string image_topic;
        };
        
    } // namespace simulation

} // namespace tkp
