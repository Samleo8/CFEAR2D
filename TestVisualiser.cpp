#include <Eigen/Geometry>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "OdometryVisualiser.hpp"
#include "RadarFeed.hpp"
#include "RadarImage.hpp"

namespace fs = std::filesystem;

int main(int argc, char **argv) {
    std::string dataset = "0";
    int startFrame = 0, endFrame = -1;
    if (argc < 2 || argc > 5) {
        printf("Usage: %s [dataset [startFrame [endFrame "
               "[0/1:outputToFile]]]]\n", argv[0]);
        return 1;
    }

    bool outputToFile = false;

    if (argc >= 2) {
        dataset = argv[1];
        if (argc >= 3) {
            startFrame = atoi(argv[2]);

            if (argc >= 4) {
                endFrame = atoi(argv[3]);

                if (argc >= 5) {
                    outputToFile = (bool)atoi(argv[4]);
                }
            }
        }
    }

    // Default data folder
    fs::path dataPath(".");
    dataPath /= "data";
    dataPath /= dataset;

    const std::string title = "Odometry Visualiser (" + dataset + ")";

    RadarFeed feed(dataPath);

    const int outputFile = (outputToFile) ? std::stoi(dataset) : -1;
    feed.run(startFrame, endFrame, !outputToFile, title, outputFile);

    printf("Done!\n");

    OdometryVisualiser vis;
    if (feed.getVisualiser(vis)) {
        vis.displayMessage("Done! Press Q or Esc to quit.", 0);
        vis.display();
    }

    unsigned char quits[] = { 'q', vis.KEYCODE_ESC };
    vis.waitForKeypress(quits, sizeof(quits));

    // Run python file with execve
    if (outputToFile) {
        // Default output saving
        fs::path outputBasePath = fs::current_path();
        outputBasePath /= "raw_output";

        std::string outputTextFileName =
            std::to_string(outputFile) + "_" + std::to_string(startFrame) +
            "_" + ((endFrame < 0) ? "end" : std::to_string(endFrame));

        std::string argv = "python \"" +
                           (outputBasePath / "txt_to_json.py").string() + "\" \"" +
                           (outputBasePath / outputTextFileName).string() + ".txt\"";

        std::string argv2 = "python \"" +
                            (outputBasePath / "error_check.py").string() + "\" \"" +
                            (outputBasePath / outputTextFileName).string() + ".json\"";

        if (system(argv.c_str()) < 0) {
            perror("system error");
            _exit(EXIT_FAILURE);
        }
        else {
            printf("Ran script: %s", argv.c_str());
        }

        if (system(argv2.c_str()) < 0) {
            perror("system error");
            _exit(EXIT_FAILURE);
        }
        else {
            printf("Ran script: %s\n", argv2.c_str());
        }
    }

    return 0;
}