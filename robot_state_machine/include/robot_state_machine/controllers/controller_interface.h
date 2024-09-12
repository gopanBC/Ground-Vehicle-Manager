#ifndef CONTROLLER_INTERFACE_H
#define CONTROLLER_INTERFACE_H

#include "../utilities/data/vehicle_state.h"

#include <vector>

class ControllerInterface {
public:
    virtual ~ControllerInterface() = default;
    virtual void start(const std::vector<double>& path_points_x, const std::vector<double>& path_points_y, const std::vector<double>& path_points_yaw) = 0;
    virtual void update(const VehicleState& vehicle_state) = 0;
};

#endif // CONTROLLER_INTERFACE_H
