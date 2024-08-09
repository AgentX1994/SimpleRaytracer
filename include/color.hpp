#pragma once

#include "math.hpp"
#include <cstdint>
#include <concepts>
#include <iostream>

namespace raytracer
{
    template <std::floating_point T>
    struct Color
    {
        T r;
        T g;
        T b;

        Color<T> SaturateColor() {
            return Color<T>{
                Saturate(r),
                Saturate(g),
                Saturate(b)
            };
        }
    };

    template <std::floating_point T>
    Color<T> operator+(Color<T> left, Color<T> right)
    {
        return Color<T>{
            left.r + right.r,
            left.g + right.g,
            left.b + right.b};
    }

    template <std::floating_point T>
    Color<T>& operator+=(Color<T>& left, Color<T> right)
    {
        left.r += right.r;
        left.g += right.g;
        left.b += right.b;
        return left;
    }

    template <std::floating_point T>
    Color<T> operator*(Color<T> left, Color<T> right)
    {
        return Color<T>{
            left.r * right.r,
            left.g * right.g,
            left.b * right.b};
    }

    template <std::floating_point T>
    Color<T> operator*(T left, Color<T> right)
    {
        return Color<T>{
            left * right.r,
            left * right.g,
            left * right.b};
    }

    template <std::floating_point T>
    Color<T> operator*(Color<T> left, T right)
    {
        return Color<T>{
            left.r * right,
            left.g * right,
            left.b * right};
    }

    template <std::floating_point T>
    Color<T> operator/(Color<T> left, T right)
    {
        return Color<T>{
            left.r / right,
            left.g / right,
            left.b / right};
    }

    template <std::floating_point T>
    std::ostream& operator<<(std::ostream& str, Color<T> c)
    {
        str << c.r << ", " << c.g << ", " << c.b;
        return str;
    }
}
