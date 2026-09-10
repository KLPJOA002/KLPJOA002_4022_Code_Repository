#pragma once

#include <cmath>

// Per-scan display colors, shared by the 3D view and the legend panel so a
// row's color always matches the points it refers to. Header-only: it is
// pure arithmetic and both users want it inlined into their draw loops.
namespace ScanColors
{

inline void hsvToRgb(double h, double s, double v, double& r, double& g, double& b)
{
    const int i = static_cast<int>(h * 6.0);
    const double f = h * 6.0 - i;
    const double p = v * (1.0 - s);
    const double q = v * (1.0 - f * s);
    const double t = v * (1.0 - (1.0 - f) * s);
    switch (i % 6)
    {
        case 0: r = v; g = t; b = p; break;
        case 1: r = q; g = v; b = p; break;
        case 2: r = p; g = v; b = t; break;
        case 3: r = p; g = q; b = v; break;
        case 4: r = t; g = p; b = v; break;
        default: r = v; g = p; b = q; break;
    }
}

// Golden-angle hue stepping keeps consecutive indices visually distinct
// even for many scans, unlike a linear hue ramp which makes neighbours
// nearly identical once you have more than a handful of scans.
inline void forIndex(int index, double& r, double& g, double& b)
{
    const double hue = std::fmod(index * 0.61803398875, 1.0);
    hsvToRgb(hue, 0.75, 0.95, r, g, b);
}

// Flagged scans keep enough of their own hue to stay identifiable while
// clearly reading as "look at this one".
inline void tintFlagged(double& r, double& g, double& b)
{
    r = 0.5 * r + 0.5;
    g *= 0.5;
    b *= 0.5;
}

} // namespace ScanColors
