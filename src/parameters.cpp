#include "parameters.h"

namespace tkp
{
    namespace simulation
    {
        Parameters readParameters(const std::string parameters_file)
        {
            Parameters parameters;

            std::fstream file_handler(parameters_file, std::ios::in);

            if (file_handler.is_open())
            {
                std::string line;

                while (std::getline(file_handler, line))
                {
                    if (size_t pos = line.find(':'); pos != std::string::npos)
                    {
                        const std::string p_name = line.substr(0, pos);
                        const std::string p_value = line.substr(pos + 2, line.length() - pos - 2);

                        parameters[p_name] = p_value;
                    }
                }

                file_handler.close();
            }
            else
            {
                std::cout << "Could not open parameter file!" << std::endl;
            }

            return parameters;
        }


        void XYControllerParameters::setParameters(Parameters& parameters)
        {
            linear_velocity_big = std::stod(parameters["linear velocity big"]);
            linear_velocity     = std::stod(parameters["linear velocity"]);
            x_tolerance         = std::stod(parameters["x tolerance"]);
            y_tolerance         = std::stod(parameters["y tolerance"]);
        }


        void XYawControllerParameters::setParameters(Parameters& parameters)
        {
            angular_p          = std::stod(parameters["angular p"]);
            linear_velocity    = std::stod(parameters["linear velocity"]);
            distance_tolerance = std::stod(parameters["distance tolerance"]);
            angle_tolerance    = std::stod(parameters["angle tolerance"]);
        
        }


        void RecorderParameters::setParameters(Parameters& parameters)
        {
            odometry_topic = parameters["odometry topic"];
            gnss_topic     = parameters["gnss topic"];
            imu_topic      = parameters["imu topic"];
            image_topic    = parameters["image topic"];
        }
    
    }
}
