#pragma once

#include <thread>
#include <vector>

namespace raytracer
{
    class Raytracer
    {
    public:
        Raytracer(int width, int height);
        ~Raytracer();

        void StartTrace();
        void StopTrace();

        const std::vector<uint8_t> &GetPixels();

    private:
        int width;
        int height;

        void ThreadTraceScene();

        std::vector<uint8_t> pixel_data;
        std::thread tracing_thread;
    };
}
