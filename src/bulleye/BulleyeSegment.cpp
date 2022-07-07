//
// Created by Louis on 21/09/2021.
//

#include "BulleyeSegment.h"

BulleyeSegment::BulleyeSegment(double start_radius_, double radius_, double start_angle_, double angle_)
    : start_radius(start_radius_), radius(radius_), start_angle(start_angle_), angle(angle_)
{}

bool BulleyeSegment::contains(double a, double r) const
{
    if (r < start_radius || r > start_radius + radius)
        return false;
    if ((a < start_angle || a > start_angle + angle) && (a < start_angle+BE_PI*2 || a > start_angle+BE_PI*2 + angle))
        return false;
    return true;
}
