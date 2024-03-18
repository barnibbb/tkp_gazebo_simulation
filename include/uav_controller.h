#include "vehicle_controller.h"

namespace tkp
{
    namespace simulation
    {
        class GazeboUAVController : public VehicleController
        {
        public:
            GazeboUAVController(Parameters& parameters);

            geometry_msgs::Twist computeVelocity(const State& pose) override;

        private:
            XYControllerParameters m_parameters;
        };
    
    } // namespace simulation

} // namespace tkp

