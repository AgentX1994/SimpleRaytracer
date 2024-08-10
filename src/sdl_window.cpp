#include "sdl_window.hpp"

#include <SDL_surface.h>
#include <iostream>

namespace raytracer
{

    SdlWindow::SdlWindow(int width, int height)
    {
        if (SDL_Init(SDL_INIT_VIDEO) < 0)
        {
            SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Couldn't initialize SDL: %s", SDL_GetError());
            throw new SdlError("Could't initialize sdl");
        }

        if (SDL_CreateWindowAndRenderer(width, height, SDL_WINDOW_RESIZABLE, &window, &renderer))
        {
            SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Couldn't create window and renderer: %s", SDL_GetError());
            throw new SdlError("Could't create window and renderer");
        }
    }

    SdlWindow::~SdlWindow()
    {
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);

        SDL_Quit();
    }

    SDL_Event SdlWindow::PollEvent()
    {
        SDL_Event event;

        SDL_PollEvent(&event);
        return event;
    }

    SDL_Texture *SdlWindow::MakeTexture(int width, int height)
    {
        auto tex = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_STATIC, width, height);
        return tex;
    }

    void SdlWindow::UpdateTexture(SDL_Texture *texture, int width, int height, const std::vector<uint8_t> &pixels)
    {
        SDL_UpdateTexture(texture, NULL, pixels.data(), 4*width);
    }

    void SdlWindow::DestroyTexture(SDL_Texture *texture)
    {
        SDL_DestroyTexture(texture);
    }

    void SdlWindow::Clear()
    {
        SDL_SetRenderDrawColor(renderer, 0x00, 0x00, 0x00, 0x00);
        SDL_RenderClear(renderer);
    }

    void SdlWindow::DrawTexture(SDL_Texture *texture)
    {
        SDL_RenderCopy(renderer, texture, NULL, NULL);
    }

    void SdlWindow::Present()
    {
        SDL_RenderPresent(renderer);
    }
}
