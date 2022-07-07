#include <iostream>
#include <filesystem>

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

    // TODO

    return EXIT_SUCCESS;
}
