#include <iostream>

#include "sdl_window.hpp"
#include "raytracer.hpp"

using namespace raytracer;

constexpr int WIDTH = 800;
constexpr int HEIGHT = 600;

int main(int argc, char **argv)
{
    std::cout << "Hello, world!" << std::endl;
    for (int i = 0; i < argc; ++i)
    {
        std::cout << '\t' << argv[i] << std::endl;
    }
    Raytracer rt(WIDTH, HEIGHT);
    // Raytrace
    auto tex_data = rt.TraceScene();

    SdlWindow window(WIDTH, HEIGHT);
    auto tex = window.MakeTexture(WIDTH, HEIGHT, tex_data.data());

    while (true)
    {
        auto event = window.PollEvent();
        if (event.type == SDL_QUIT)
        {
            break;
        }
        else if (event.type == SDL_KEYDOWN)
        {
            if (event.key.keysym.sym == SDLK_ESCAPE)
            {
                break;
            }
        }
        window.Clear();
        window.DrawTexture(tex);
        window.Present();
    }

    window.DestroyTexture(tex);
}