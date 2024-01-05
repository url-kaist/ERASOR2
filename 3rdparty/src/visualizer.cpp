#include "visualizer.h"

Visualizer::Visualizer(){};

Visualizer::~Visualizer(){};

void Visualizer::progressBar(int current, int maximum, const std::string &filename, bool saveFlag)
{
    static bool firstCall = true;

    // Determine the status message
    std::string statusMessage;
    if (!filename.empty())
    {
        if (!timingStarted)
        {
            startTime = std::chrono::steady_clock::now(); // Start timing
            timingStarted = true;
        }
        statusMessage = saveFlag ? "Saving: " : "Processing: ";
    }
    else
    {
        statusMessage = "Loading the trajectory";
    }

    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<float> elapsed = timingStarted ? now - startTime : std::chrono::seconds(0);

    float percentage = timingStarted ? static_cast<float>(current) / maximum : 0.0f;
    int lpad = static_cast<int>(percentage * PBWIDTH);

    float remainingTime = (elapsed.count() / percentage) - elapsed.count();
    float totalTime = elapsed.count() + remainingTime;

    // Format the elapsed and total times into hours, minutes, and seconds
    int elapsedHours = static_cast<int>(elapsed.count()) / 3600;
    int elapsedMinutes = (static_cast<int>(elapsed.count()) / 60) % 60;
    int elapsedSeconds = static_cast<int>(elapsed.count()) % 60;

    int totalHours = static_cast<int>(totalTime) / 3600;
    int totalMinutes = (static_cast<int>(totalTime) / 60) % 60;
    int totalSeconds = static_cast<int>(totalTime) % 60;
    if (!firstCall)
    {
        // Move the cursor up two lines to overwrite the previous output
        printf("\033[2A");
    }

    // Clear the current line and move to the beginning
    printf("\r\033[K");

    // Print the status message on the first line
    std::cout << cyan << statusMessage << filename << reset << "\n";

    // Clear the next line and move to the beginning
    printf("\r\033[K");

    // Print the progress bar and time information on the second line
    if (timingStarted)
    {
        float binsPerSec = static_cast<float>(current / elapsed.count());
        std::cout << std::fixed << std::setprecision(2) << percentage * 100 << "% ["
                  << std::string(lpad, '=') << std::string(PBWIDTH - lpad, ' ')
                  << "] " << current << "/" << maximum
                  << " [" << std::setfill('0') << std::setw(2) << elapsedHours << ":"
                  << std::setw(2) << elapsedMinutes << ":" << std::setw(2) << elapsedSeconds
                  << "<" << std::setw(2) << totalHours << ":" << std::setw(2) << totalMinutes
                  << ":" << std::setw(2) << totalSeconds << ", " << binsPerSec << "bin/s]"
                  << "\n";
    }
    else
    {
        std::cout << "--% [" << std::string(PBWIDTH, ' ') << "] "
                  << "--/-- [--:--:--<--:--:--, --bin/s]"
                  << "\n";
    }

    firstCall = false;
    fflush(stdout);
}

std::string Visualizer::getInput(const std::string &prompt)
{
    std::cout << prompt;
    std::string input;
    std::getline(std::cin, input);
    return input;
}

std::string Visualizer::lidarTypeToString(LiDARType lidarType)
{
    switch (lidarType)
    {
    case OUSTER:
        return "Ouster";
    case VELODYNE:
        return "Velodyne";
    case LIVOX:
        return "Livox";
    case AEVA:
        return "Aeva";
    default:
        return "Unknown";
    }
}
