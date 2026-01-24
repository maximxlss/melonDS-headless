#ifndef MELONDS_HEADLESS_SCREENSHOT_H
#define MELONDS_HEADLESS_SCREENSHOT_H

#include <cstdio>
#include <string>
#include <vector>

#include "types.h"

#include <lodepng.h>

inline bool WritePNG(const std::string& path, const melonDS::u32* top, const melonDS::u32* bottom, int width, int height)
{
    const int totalHeight = height * 2;
    std::vector<unsigned char> image;
    image.resize(static_cast<size_t>(width) * static_cast<size_t>(totalHeight) * 4);
    for (int y = 0; y < totalHeight; ++y)
    {
        const melonDS::u32* src = nullptr;
        int srcY = 0;
        if (y < height)
        {
            src = top;
            srcY = y;
        }
        else
        {
            src = bottom;
            srcY = y - height;
        }

        unsigned char* dst = image.data()
            + (static_cast<size_t>(y) * static_cast<size_t>(width) * 4);
        const melonDS::u32* rowPixels = src + (srcY * width);
        for (int x = 0; x < width; ++x)
        {
            melonDS::u32 pixel = rowPixels[x];
            *dst++ = static_cast<unsigned char>((pixel >> 16) & 0xFF);
            *dst++ = static_cast<unsigned char>((pixel >> 8) & 0xFF);
            *dst++ = static_cast<unsigned char>(pixel & 0xFF);
            *dst++ = 0xFF;
        }
    }

    const unsigned error = lodepng::encode(path, image, width, totalHeight);
    if (error != 0)
    {
        std::fprintf(stderr, "PNG encode failed: %s\n", lodepng_error_text(error));
        return false;
    }
    return true;
}

#endif
