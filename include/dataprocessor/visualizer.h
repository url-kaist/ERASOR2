#ifndef VISUALIZER_H
#define VISUALIZER_H

#include "utility.h"
#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"
#define PBWIDTH 60

class Visualizer
{
public:
    Visualizer();
    ~Visualizer();

    std::chrono::steady_clock::time_point startTime;
    bool timingStarted = false;

    void progressBar(int current, int maximum, const std::string &filename, bool saveFlag);

    // for utilize getInput function in another package
    // search about "Instantiation-style polymorphism" in C++
    template <typename T>
    T getInput(const std::string &prompt, T defaultValue)
    {
        std::cout << prompt;
        T input;
        if (std::cin >> input)
        {
            return input;
        }
        else
        {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            return defaultValue;
        }
    }

    std::string getInput(const std::string &prompt);

    std::string lidarTypeToString(LiDARType lidarType);
};

#endif // VISUALIZER_H