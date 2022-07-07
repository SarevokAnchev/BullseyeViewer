//
// Created by Louis on 21/09/2021.
//

#pragma once

#include <vector>
#include <string>

#include "BulleyeSegment.h"

enum class BulleyeConfig {
    bulleye_16_segments,
    bulleye_17_segments,
    bulleye_18_segments
};

class Bulleye {
private:
    const double BE_PI = 3.14159265358979323846;
    double apex[3] = {0, 0, 0};
    double axis[3] = {1, 0, 0};
    double anterior_axis[3] = {0, 1, 0};

    double height = 1.;

    int n_rings = 4;
    std::vector<int> n_segments = {1, 4, 6, 6};

    std::vector<BulleyeSegment> segments;

    std::vector<std::vector<std::pair<double, double>>> scalars_coords;
    std::vector<std::vector<double>> scalars_values;
    std::vector<std::string> scalars_labels;


public:
    Bulleye(BulleyeConfig cfg, const double* apex_, const double* axis_, double height, const double* anterior_axis);
    Bulleye(int n_rings_, const int* n_segments_, const double* apex_, const double* axis_, double height, const double* anterior_axis_);
    Bulleye(BulleyeConfig cfg, const double * apex, const double* base, const double* front);

    [[nodiscard]] const BulleyeSegment* is_inside(double x, double y, double z) const;

    void polar_coordinates(double x, double y, double z, double& angle, double& radius) const;

    void add_scalars(const std::string& label_, std::vector<std::pair<double, double>> coords_, std::vector<double> values_);

    void add_scalars(const std::string& label_, const std::vector<std::tuple<double, double, double>>& coords_, std::vector<double> values_);

    void get_scalars(const std::string& label_, std::vector<std::pair<double, double>>& coords_, std::vector<double>& values_);

    void get_scalars(int index, std::vector<std::pair<double, double>>& coords_, std::vector<double>& values_);

    [[nodiscard]] inline std::vector<std::string> get_labels() const { return scalars_labels; }

    [[nodiscard]] bool scalars_exists(const std::string& label_) const;

    [[nodiscard]] inline size_t n_labels() const { return scalars_labels.size(); }

    [[nodiscard]] inline std::vector<int> get_n_segments() const { return n_segments; }

    [[nodiscard]] std::vector<BulleyeSegment> get_segments() const { return segments; }

    [[nodiscard]] double get_height() const { return height; }

    [[nodiscard]] std::vector<double> get_apex() const { return {apex[0], apex[1], apex[2]}; }
    void get_apex(double* apex_) const { apex_[0] = apex[0]; apex_[1] = apex[1]; apex_[2] = apex[2]; }
    [[nodiscard]] std::vector<double> get_axis() const { return {axis[0], axis[1], axis[2]}; }
    void get_axis(double* axis_) const { axis_[0] = axis[0]; axis_[1] = axis[1]; axis_[2] = axis[2]; }
    [[nodiscard]] std::vector<double> get_anterior_axis() const { return {anterior_axis[0], anterior_axis[1], anterior_axis[2]}; }
    void get_anterior_axis(double* anterior_axis_) const
    {
        anterior_axis_[0] = anterior_axis[0];
        anterior_axis_[1] = anterior_axis[1];
        anterior_axis_[2] = anterior_axis[2];
    }
};
