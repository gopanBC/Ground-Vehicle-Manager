#include "../../include/robot_state_machine/utilities/waypoint_reader.h"

WPReader::WPReader(const std::string& filename) : filename(filename) {}

void WPReader::readWaypoints() {
    std::ifstream file(filename);
    std::string line;

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::vector<double> point;
        double value;
        while (ss >> value) {
            point.push_back(value);
            if (ss.peek() == ',') ss.ignore();
        }

        // Add the points to respective vectors
        path_points_x.push_back(point[0]);
        path_points_y.push_back(point[1]);
        path_points_yaw.push_back(point[2]);
    }

    // Optionally print the loaded waypoints
    std::cout << "Loaded " << path_points_x.size() << " waypoints.\n";
}

const std::vector<double>& WPReader::getPathPointsX() const {
    return path_points_x;
}

const std::vector<double>& WPReader::getPathPointsY() const {
    return path_points_y;
}

const std::vector<double>& WPReader::getPathPointsYaw() const {
    return path_points_yaw;
}
