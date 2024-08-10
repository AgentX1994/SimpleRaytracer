#pragma once

#include <stdexcept>
#include <vector>

#include <SDL.h>

namespace raytracer
{
    class SdlError : public std::runtime_error
    {
    public:
        SdlError(const char *msg) : std::runtime_error(msg) {}
    };

    class SdlWindow
    {
    public:
        SdlWindow(int width, int height);
        ~SdlWindow();

        SDL_Event PollEvent();

        SDL_Texture *MakeTexture(int width, int height);
        void UpdateTexture(SDL_Texture *texture, int width, int height, const std::vector<uint8_t> &pixels);
        void DestroyTexture(SDL_Texture *texture);
        void Clear();
        void DrawTexture(SDL_Texture *texture);
        void Present();

    private:
        SDL_Window *window;
        SDL_Renderer *renderer;
        SDL_Surface *surface;
    };
}
