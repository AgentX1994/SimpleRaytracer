#include <iostream>
#include <vector>

#include "raytracer.hpp"
#include "sdl_window.hpp"

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
    rt.StartTrace();

    SdlWindow window(WIDTH, HEIGHT);
    auto tex = window.MakeTexture(WIDTH, HEIGHT);

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

        window.UpdateTexture(tex, WIDTH, HEIGHT, rt.GetPixels());

        window.Clear();
        window.DrawTexture(tex);
        window.Present();
    }

    rt.StopTrace();

    window.DestroyTexture(tex);
}
