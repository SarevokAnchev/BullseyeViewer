//
// Created by Louis on 21/09/2021.
//

#include <bullseye/Bullseye.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

Bullseye::Bullseye(BullseyeConfig cfg, const double *apex_, const double *axis_, double height_, const double *anterior_axis_)
{
    switch (cfg) {
        case BullseyeConfig::bullseye_16_segments:
            n_rings = 3;
            n_segments = {4, 6, 6};
            break;
        case BullseyeConfig::bullseye_17_segments:
            n_rings = 4;
            n_segments = {1, 4, 6, 6};
            break;
        case BullseyeConfig::bullseye_18_segments:
            n_rings = 3;
            n_segments = {6, 6, 6};
            break;
    }

    height = height_;

    for (int i = 0; i < 3; i++) apex[i] = apex_[i];
    for (int i = 0; i < 3; i++) axis[i] = axis_[i];
    for (int i = 0; i < 3; i++) anterior_axis[i] = anterior_axis_[i];

    double norm = sqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);
    for (int i = 0; i < 3; i++) {
        axis[i] = axis[i]/norm;
    }
    double ant_norm = sqrt(anterior_axis[0]*anterior_axis[0] + anterior_axis[1]*anterior_axis[1] + anterior_axis[2]*anterior_axis[2]);
    for (int i = 0; i < 3; i++) {
        anterior_axis[i] = anterior_axis[i]/ant_norm;
    }

    double ring_radius = 1./n_rings;
    for (int i = 0; i < n_rings; i++) {
        double angle_size = BE_PI*2/n_segments[i];
        double ring_start_radius = (double)i/n_rings;
        for (int j = 0; j < n_segments[i]; j++) {
            segments.emplace_back(BullseyeSegment(
                    ring_start_radius,
                    ring_radius,
                    -angle_size/2 + j*angle_size,
                    angle_size
            ));
        }
    }
}

Bullseye::Bullseye(int n_rings_, const int *n_segments_, const double *apex_, const double *axis_,
                   double height_, const double *anterior_axis_)
{
    n_rings = n_rings_;

    height = height_;

    n_segments.clear();
    for (int i = 0; i < n_rings; i++) n_segments.push_back(n_segments_[i]);

    for (int i = 0; i < 3; i++) apex[i] = apex_[i];
    for (int i = 0; i < 3; i++) axis[i] = axis_[i];
    for (int i = 0; i < 3; i++) anterior_axis[i] = anterior_axis_[i];

    double norm = sqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);
    for (int i = 0; i < 3; i++) {
        axis[i] = axis[i]/norm;
    }
    double ant_norm = sqrt(anterior_axis[0]*anterior_axis[0] + anterior_axis[1]*anterior_axis[1] + anterior_axis[2]*anterior_axis[2]);
    for (int i = 0; i < 3; i++) {
        anterior_axis[i] = anterior_axis[i]/ant_norm;
    }

    double ring_radius = 1./n_rings;
    for (int i = 0; i < n_rings; i++) {
        double angle_size = BE_PI*2/n_segments[i];
        double ring_start_radius = (double)i/n_rings;
        for (int j = 0; j < n_segments[i]; j++) {
            segments.emplace_back(BullseyeSegment(
                    ring_start_radius,
                    ring_radius,
                    -angle_size/2 + j*angle_size,
                    angle_size
            ));
        }
    }
}

Bullseye::Bullseye(BullseyeConfig cfg, const double *apex_, const double *base_, const double *front_)
{
    switch (cfg) {
        case BullseyeConfig::bullseye_16_segments:
            n_rings = 3;
            n_segments = {4, 6, 6};
            break;
        case BullseyeConfig::bullseye_17_segments:
            n_rings = 4;
            n_segments = {1, 4, 6, 6};
            break;
        case BullseyeConfig::bullseye_18_segments:
            n_rings = 3;
            n_segments = {6, 6, 6};
            break;
    }

    double long_axis[] = {
            base_[0] - apex_[0],
            base_[1] - apex_[1],
            base_[2] - apex_[2]
    };

    height = sqrt(long_axis[0]*long_axis[0] + long_axis[1]*long_axis[1] + long_axis[2]*long_axis[2]);
    for (int i = 0; i < 3; i++) {
        apex[i] = apex_[i];
        axis[i] = long_axis[i]/height;
    }

    // find anterior axis from front point projection
    double front_vec[] = {
            front_[0] - apex_[0],
            front_[1] - apex_[1],
            front_[2] - apex_[2]
    };
    double dot_prod = long_axis[0]*front_vec[0] + long_axis[1]*front_vec[1] + long_axis[2]*front_vec[2];
    double front_proj_height = dot_prod/height;
    double proj_front[] = {
            apex[0] + front_proj_height*axis[0],
            apex[1] + front_proj_height*axis[1],
            apex[2] + front_proj_height*axis[2]
    };
    double front_point_axis[] = {
            front_[0] - proj_front[0],
            front_[1] - proj_front[1],
            front_[2] - proj_front[2]
    };
    double front_axis_norm = sqrt(front_point_axis[0]*front_point_axis[0] + front_point_axis[1]*front_point_axis[1] + front_point_axis[2]*front_point_axis[2]);
    for (int i = 0; i < 3; i++) { anterior_axis[i] = front_point_axis[i]/front_axis_norm; }

    double ring_radius = 1./n_rings;
    for (int i = 0; i < n_rings; i++) {
        double angle_size = BE_PI*2/n_segments[i];
        double ring_start_radius = (double)i/n_rings;
        for (int j = 0; j < n_segments[i]; j++) {
            segments.emplace_back(BullseyeSegment(
                    ring_start_radius,
                    ring_radius,
                    -angle_size/2 + j*angle_size,
                    angle_size
            ));
        }
    }
}

const BullseyeSegment* Bullseye::is_inside(double x, double y, double z) const
{
    double radius, angle;
    polar_coordinates(x, y, z, angle, radius);
    if (radius > 1. || radius < 0.) return nullptr;
    for (const auto & segment : segments) {
        if (segment.contains(angle, radius))
            return &segment;
    }
    return nullptr;
}

void Bullseye::polar_coordinates(double x, double y, double z, double& angle_, double& radius_) const
{
    double point_vec[3] = {x - apex[0], y - apex[1], z - apex[2]};
    double dot_prod = axis[0]*point_vec[0] + axis[1]*point_vec[1] + axis[2]*point_vec[2];

    double point_height = dot_prod/1.;
    radius_ = point_height/height;

    double proj_point[3] = {
            apex[0] + axis[0]*point_height,
            apex[1] + axis[1]*point_height,
            apex[2] + axis[2]*point_height
    };
    double point_normal[3] = {
            x - proj_point[0],
            y - proj_point[1],
            z - proj_point[2]
    };
    double normal_dot_prod = anterior_axis[0]*point_normal[0] + anterior_axis[1]*point_normal[1] + anterior_axis[2]*point_normal[2];

    double normal_cross_prod[3] = {
            anterior_axis[0]*point_normal[1] - anterior_axis[1]*point_normal[0],
            anterior_axis[1]*point_normal[2] - anterior_axis[2]*point_normal[1],
            anterior_axis[2]*point_normal[0] - anterior_axis[0]*point_normal[2]
    };
    double cp_dot_prod = normal_cross_prod[0]*axis[0] + normal_cross_prod[1]*axis[1] + normal_cross_prod[2]*axis[2];
    angle_ = atan2(cp_dot_prod, normal_dot_prod);
    if (angle_ < 0) {
        angle_ = 2*BE_PI + angle_;
    }
}

void Bullseye::add_scalars(const std::string& label_, std::vector<std::pair<double, double>> coords_, std::vector<double> values_)
{
    for (auto & scalars_label : scalars_labels) {
        if (scalars_label == label_) {
            return;
        }
    }
    scalars_labels.emplace_back(label_);
    scalars_coords.emplace_back(std::move(coords_));
    scalars_values.emplace_back(std::move(values_));
}

void Bullseye::add_scalars(const std::string &label_, const std::vector<std::tuple<double, double, double>>& coords_,
                           std::vector<double> values_)
{
    for (auto & scalars_label : scalars_labels) {
        if (scalars_label == label_) {
            return;
        }
    }
    scalars_labels.emplace_back(label_);
    std::vector<std::pair<double, double>> polar_coords;
    for (auto& c : coords_) {
        double angle, radius;
        polar_coordinates(std::get<0>(c), std::get<1>(c), std::get<2>(c), angle, radius);
        polar_coords.emplace_back(angle, radius);
    }
    scalars_coords.emplace_back(std::move(polar_coords));
    scalars_values.emplace_back(std::move(values_));
}

void Bullseye::get_scalars(const std::string &label_, std::vector<std::pair<double, double>>& coords_,
                           std::vector<double>& values_)
{
    int idx = -1;
    for (int i = 0; i < scalars_labels.size(); i++) {
        if (scalars_labels[i] == label_) {
            idx = i;
            break;
        }
    }
    try {
        get_scalars(idx, coords_, values_);
    }
    catch(std::exception& exc) {
        throw;
    }
}

void Bullseye::get_scalars(int index, std::vector<std::pair<double, double>>& coords_,
                           std::vector<double>& values_)
{
    if (index < 0 || index >= n_labels()) {
        throw std::runtime_error("Invalid index");
    }
    else {
        coords_ = scalars_coords[index];
        values_ = scalars_values[index];
    }
}

bool Bullseye::scalars_exists(const std::string &label_) const
{
    return std::any_of(scalars_labels.begin(), scalars_labels.end(), [label_](auto& l) { return l == label_; });
}

