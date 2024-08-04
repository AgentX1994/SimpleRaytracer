#pragma once

#include <vector>

#include "math.hpp"

namespace raytracer
{
    class Raytracer
    {
    public:
        Raytracer(int width, int height);
        ~Raytracer();

        std::vector<uint8_t> TraceScene();

    private:
        int width;
        int height;
    };
}