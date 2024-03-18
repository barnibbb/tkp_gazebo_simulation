#include "vehicle_controller.h"

namespace tkp
{
    namespace simulation
    {
        class GazeboUGVController : public VehicleController
        {
        public:
            GazeboUGVController(Parameters& parameters);

            geometry_msgs::Twist computeVelocity(const State& pose) override;

        private:
            XYawControllerParameters m_parameters;
        };

    } // namespace simulation

} // namespace tkp

