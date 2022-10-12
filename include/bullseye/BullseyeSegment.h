//
// Created by Louis on 21/09/2021.
//

#pragma once

class BullseyeSegment {
private:
    const double BE_PI = 3.14159265358979323846;
    double start_radius = 0;
    double radius = 1;

    double start_angle = 0;
    double angle = BE_PI/2;

public:
    BullseyeSegment(double start_radius_, double radius_, double start_angle_, double angle_);

    [[nodiscard]] bool contains(double a, double r) const;

    [[nodiscard]] inline double get_start_radius() const { return start_radius; }
    [[nodiscard]] inline double get_height() const { return radius; }

    [[nodiscard]] inline double get_start_angle() const { return start_angle; }
    [[nodiscard]] inline double get_angle_span() const { return angle; }
};
