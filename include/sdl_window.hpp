#pragma once

#include <stdexcept>

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

        SDL_Texture *MakeTexture(int width, int height, const uint8_t *data);
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