//
// Created by Louis on 22/09/2021.
//

#include "BulleyeDisplay.h"

#include <utility>

#include <opencv2/opencv.hpp>

#include "../kdtree/KDTree.h"

BulleyeDisplay::BulleyeDisplay(std::shared_ptr<Bulleye> be_)
    : be(std::move(be_))
{}

cv::Mat BulleyeDisplay::get_canva_scalars_interp(const std::string &label_) const
{
    std::vector<std::pair<double, double>> polar_coords;
    std::vector<double> values;
    be->get_scalars(label_, polar_coords, values);

    std::vector<std::vector<double>> coords(polar_coords.size());
    for (int i = 0; i < coords.size(); i++) {
        std::vector<double> point(2);
        // -r * sin(theta) -> [-1, 1]
        point[0] = -polar_coords[i].second*sin(polar_coords[i].first);
        // r * cos(theta) -> [-1, 1]
        point[1] = polar_coords[i].second*cos(polar_coords[i].first);

        coords[i] = point;
    }

    KDTree<double> kd;
    kd.add_nodes(coords, values, true);

    cv::Mat interp_scalars(2*radius, 2*radius, CV_64F);
    interp_scalars.forEach<double>([&](double& value, const int position[]) -> void {
        double y = (double)(position[0] - radius)/radius;
        double x = (double)(position[1] - radius)/radius;

        auto nearest = kd.get_closest_node({x, y});
        value = nearest->data();
    });
    return interp_scalars;
}

cv::Mat BulleyeDisplay::get_canva_from_scalars(const cv::Mat &scalars, vtkSmartPointer<vtkLookupTable> lut) const
{
    cv::Mat canva(scalars.rows, scalars.cols, CV_8UC3);

    canva.forEach<uint8_t>([&](uint8_t& value, const int position[]) -> void {
        double scalar = scalars.at<double>(position);
        double color[3];
        lut->GetColor(scalar, color);
        canva.at<cv::Point3_<uint8_t>>(position) = cv::Point3_<uint8_t>(color[2]*255, color[1]*255, color[0]*255);
    });

    return canva;
}

void BulleyeDisplay::show(const std::string &label_)
{
    // TODO
}

void BulleyeDisplay::draw_segments(cv::Mat &canva) const
{
    cv::Point center(radius - 1, radius - 1);
    auto n_segments = be->get_n_segments();
    // draw circles
    for (int i = 0; i < n_segments.size(); i++) {
        int r = (i+1)*(radius/n_segments.size());
        cv::circle(canva, center, r, {0, 0, 0}, 2);
    }

    // draw segments
    // first segment always centered
    auto segments = be->get_segments();
    for (BulleyeSegment& s: segments) {
        auto r = s.get_start_radius();
        auto a = s.get_start_angle() + BE_PI/2;
        auto l = s.get_height();
        auto x_s = r*cos(a)*radius + center.x;
        auto y_s = r*sin(a)*radius + center.y;
        auto x_e = (r + l)* cos(a)*radius + center.x;
        auto y_e = (r + l)* sin(a)*radius + center.y;
        cv::line(canva, {(int)x_s, (int)y_s}, {(int)x_e, (int)y_e}, {0, 0, 0}, 2);
    }
}

cv::Mat BulleyeDisplay::draw(const std::string &label_, vtkSmartPointer<vtkLookupTable> lut)
{
    auto scalars = get_canva_scalars_interp(label_);

    auto canva = get_canva_from_scalars(scalars, lut);

    draw_segments(canva);

    return canva;
}

void BulleyeDisplay::set_display_scalars_points(bool v_)
{}
