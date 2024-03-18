#include <cmath>

#include "uav_controller.h"

void handler(int s)
{
    exit(1);
}

namespace tkp
{
    namespace simulation
    {
        GazeboUAVController::GazeboUAVController(Parameters& parameters)
        {
            m_parameters.setParameters(parameters);
        }


        geometry_msgs::Twist GazeboUAVController::computeVelocity(const State& pose)
        {
            signal (SIGINT, handler);

            const Waypoint target = m_waypoints[m_current_target];

            const double linear_velocity = (m_current_target == 0) ? m_parameters.linear_velocity_big : m_parameters.linear_velocity;

            const double x_distance = target.x - pose.x;
            const double y_distance = target.y - pose.y;

            const double theta = std::atan2(y_distance, x_distance);

            geometry_msgs::Twist velocity;

            velocity.linear.x = (std::abs(x_distance) > m_parameters.x_tolerance) ? (linear_velocity * std::cos(theta)) : 0;

            velocity.linear.y = (std::abs(y_distance) > m_parameters.y_tolerance) ? linear_velocity * std::sin(theta) : 0;

            if (std::abs(x_distance) < m_parameters.x_tolerance && std::abs(y_distance) < m_parameters.y_tolerance)
            {
                ++m_current_target; 
            }

            return velocity;
        }
    }
}

