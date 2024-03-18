#include <fstream>
#include <sstream>

#include "vehicle_controller.h"

namespace tkp
{
    namespace simulation
    {       
        void VehicleController::readWaypoints(const std::string& waypoints_file)
        {
            std::fstream file_handler(waypoints_file, std::ios::in);

            if (file_handler.is_open())
            {
                std::string line;

                while (std::getline(file_handler, line))
                {
                    std::stringstream line_stream(line);

                    std::vector<std::string> line_data;

                    std::string data;

                    while (std::getline(line_stream, data, ' '))
                    {
                        line_data.push_back(data);
                    }

                    if (line_data.size() == 3)
                    {
                        Waypoint waypoint { std::stod(line_data[0]), std::stod(line_data[1]), std::stod(line_data[2]) };

                        m_waypoints.push_back(waypoint);
                    }
                    else
                    {
                        throw std::runtime_error { "Invalid file format" };
                    }
                }

                file_handler.close();

                m_current_target = 0;

                std::cout << m_waypoints.size() << std::endl;
            }
            else
            {
                throw std::runtime_error { "Could not open waypoint file!" };
            }
        }
    

        bool VehicleController::isRouteCompleted() const
        {
            return (m_waypoints.size() == m_current_target);
        }

    }
}