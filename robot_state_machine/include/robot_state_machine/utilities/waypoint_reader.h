#ifndef WP_READER_H
#define WP_READER_H

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

class WPReader {
public:
    WPReader(const std::string& filename);

    void readWaypoints();  // Method to read the waypoints from the file

    const std::vector<double>& getPathPointsX() const;  // Accessor for X points
    const std::vector<double>& getPathPointsY() const;  // Accessor for Y points
    const std::vector<double>& getPathPointsYaw() const;  // Accessor for Yaw points

private:
    std::string filename;

    std::vector<double> path_points_x;
    std::vector<double> path_points_y;
    std::vector<double> path_points_yaw;
};

#endif // WP_READER_H
