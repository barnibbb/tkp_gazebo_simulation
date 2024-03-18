#include <string>
#include <vector>

#include <signal.h>

#include <geometry_msgs/Twist.h>

#include "parameters.h"

namespace tkp
{
    namespace simulation
    {
        struct State { double x, y, z, roll, pitch, yaw; };

        struct Waypoint { double x, y, z; };

        class VehicleController
        {
        public:
            void readWaypoints(const std::string& waypoints_file);
            
            virtual geometry_msgs::Twist computeVelocity(const State& pose) = 0;

            bool isRouteCompleted() const;

        protected:
            std::vector<Waypoint> m_waypoints;

            unsigned int m_current_target;
        };
    
    } // namespace simulation

} // namespace tkp
