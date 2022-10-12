//
// Created by Louis on 22/09/2021.
//

#pragma once

#include <memory>

#include <opencv2/core.hpp>

#include <vtkSmartPointer.h>
#include <vtkLookupTable.h>

#include "Bullseye.h"

class BullseyeDisplay {
private:
    const double BE_PI = 3.14159265358979323846;

    std::shared_ptr<Bullseye> be;

    int radius=256;

    cv::Mat img;

    [[nodiscard]] cv::Mat get_canva_scalars_interp(const std::string& label) const;

    [[nodiscard]] cv::Mat get_canva_from_scalars(const cv::Mat& scalars, vtkSmartPointer<vtkLookupTable>) const;

    void draw_segments(cv::Mat& canva) const;

public:
    explicit BullseyeDisplay(std::shared_ptr<Bullseye> be_);

    void show(const std::string& label_);

    cv::Mat draw(const std::string& label_, vtkSmartPointer<vtkLookupTable>);

    void set_display_scalars_points(bool v_);

    inline void set_radius(int r) { radius = std::abs(r); }

    [[nodiscard]] int get_radius() const { return radius; }
};

