#include <iostream>
#include <fstream>
#include <filesystem>

#include <opencv2/opencv.hpp>

#include <nlohmann/json.hpp>

#include <bullseye/Bullseye.h>
#include <bullseye/BullseyeDisplay.h>

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cerr << "Error: Invalid number of parameters. Correct usage is:" << std::endl
                  << "\tBullseyeViewer.exe sample_bullseye_file.json" << std::endl;
        return EXIT_FAILURE;
    }

    if (!std::filesystem::exists(argv[1])) {
        std::cerr << "Error: File does not exist." << std::endl;
        return EXIT_FAILURE;
    }

    std::string path = argv[1];
    nlohmann::json j;
    try {
        std::ifstream i(path);
        i >> j;
    }
    catch (const std::exception& e) {
        std::cout << "Unable to parse file: " << e.what() << std::endl;
    }

    auto apex = j.at("apex").get<std::vector<double>>();
    double apex_arr[] = {apex[0], apex[1], apex[2]};
    auto base = j.at("base").get<std::vector<double>>();
    double base_arr[] = {base[0], base[1], base[2]};
    auto front = j.at("front").get<std::vector<double>>();
    double front_arr[] = {front[0], front[1], front[2]};
    auto coords = j.at("coords").get<std::vector<std::vector<double>>>();
    auto values = j.at("values").get<std::vector<double>>();
    auto mm = std::minmax_element(values.begin(), values.end());
    auto min = *mm.first;
    auto max = *mm.second;

    auto config = BullseyeConfig::bullseye_18_segments;
    auto be = std::make_shared<Bullseye>(config, apex_arr, base_arr, front_arr);

    std::vector<std::tuple<double, double, double>> coords_scalars;
    for (const auto& c: coords) {
        coords_scalars.emplace_back(std::make_tuple(c[0], c[1], c[2]));
    }
    be->add_scalars("Values", coords_scalars, values);

    BullseyeDisplay disp(be);

    auto lut = vtkSmartPointer<vtkLookupTable>::New();
    lut->SetTableRange(min, max);
    lut->SetHueRange(0.667, 0);
    lut->SetSaturationRange(1, 1);
    lut->SetValueRange(1, 1);
    lut->SetNanColor(0.8, 0.8, 0.8, 1);
    lut->Build();

    auto mat = disp.draw("Values", lut);

    cv::flip(mat, mat, 0);
    cv::imshow("Bullseye", mat);
    cv::waitKey();

    return EXIT_SUCCESS;
}
