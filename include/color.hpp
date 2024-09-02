#pragma once

#include <concepts>
#include <cstdint>
#include <iostream>

#include "math.hpp"

namespace raytracer
{
struct Color
{
    float r;
    float g;
    float b;

    inline Color SaturateColor()
    {
        return Color{Saturate(r), Saturate(g), Saturate(b)};
    }
};

inline Color operator+(Color left, Color right)
{
    return Color{left.r + right.r, left.g + right.g, left.b + right.b};
}

inline Color& operator+=(Color& left, Color right)
{
    left.r += right.r;
    left.g += right.g;
    left.b += right.b;
    return left;
}

inline Color operator*(Color left, Color right)
{
    return Color{left.r * right.r, left.g * right.g, left.b * right.b};
}

inline Color operator*(float left, Color right)
{
    return Color{left * right.r, left * right.g, left * right.b};
}

inline Color operator*(Color left, float right)
{
    return Color{left.r * right, left.g * right, left.b * right};
}

inline Color operator/(Color left, float right)
{
    return Color{left.r / right, left.g / right, left.b / right};
}

inline std::ostream& operator<<(std::ostream& str, Color c)
{
    str << c.r << ", " << c.g << ", " << c.b;
    return str;
}
}  // namespace raytracer
