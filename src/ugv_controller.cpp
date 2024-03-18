#include <cmath>

#include "gazebo_ugv_controller.h"

void handler(int s)
{
    exit(1);
}

namespace tkp
{
    namespace simulation
    {
        GazeboUGVController::GazeboUGVController(Parameters& parameters)
        {
            m_parameters.setParameters(parameters);
        }


        geometry_msgs::Twist GazeboUGVController::computeVelocity(const State& pose)
        {
            signal (SIGINT, handler);

            State velocity { 0, 0, 0, 0, 0, 0 };

            Waypoint target = m_waypoints[m_current_target];

            const double x_distance = target.x - pose.x;
            const double y_distance = target.y - pose.y;

            const double distance = std::sqrt(std::pow(x_distance, 2) + std::pow(y_distance, 2));

            double current_direction = pose.yaw;
            double target_direction = std::atan2(y_distance, x_distance);

            target_direction  = target_direction  < 0 ? target_direction  + 2 * M_PI : target_direction;
            current_direction = current_direction < 0 ? current_direction + 2 * M_PI : current_direction;

            double theta = std::abs(target_direction - current_direction);

            if (theta <= M_PI)
            {
                theta = target_direction > current_direction ? theta : -theta;
            }
            else
            {
                theta = target_direction > current_direction ? -(2 * M_PI - theta) : 2 * M_PI - theta;
            }

            if (distance > m_parameters.distance_tolerance)
            {
                velocity.x = m_parameters.linear_velocity;

                if (std::abs(theta) > m_parameters.angle_tolerance)
                {
                    velocity.yaw = m_parameters.angular_p * theta;
                }
                else
                {
                    ++m_current_target;
                }
            }

            return velocity;
        }
    }
}
