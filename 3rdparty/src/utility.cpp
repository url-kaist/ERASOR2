#include "utility.h"

std::string padZeros(int val, int num_digits)
{
    std::ostringstream out;
    out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
    return out.str();
}

bool isInFOV(double x, double y, double z)
{
    // Convert FOV degrees to radians
    const double halfFovX = 70.4 * M_PI / 180.0 / 2.0; // Half FOV for azimuth (x-axis)
    const double halfFovY = 77.2 * M_PI / 180.0 / 2.0; // Half FOV for elevation (y-axis)

    // Convert Cartesian (x, y, z) to Spherical Coordinates (theta, phi)
    double theta = std::atan2(y, x);                      // Azimuth angle
    double phi = std::atan2(z, std::sqrt(x * x + y * y)); // Elevation angle

    // Check if within FOV
    return std::abs(theta) <= halfFovX && std::abs(phi) <= halfFovY;
}

float euclidean_distance(Point3D p1, Point3D p2)
{
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}