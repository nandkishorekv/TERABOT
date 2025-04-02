#include <stdint.h>
#include <iostream>
#include <cstring>
#include <fstream>
#include <chrono>
#include <thread>
#include <math.h>
#include <unistd.h>
#include "MLX90641_API.h"

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <SDL2/SDL_render.h>

#define MLX_I2C_ADDR 0x33

#define SENSOR_W 12
#define SENSOR_H 16

// Valid frame rates are 1, 2, 4, 8, 16, 32, and 64
#define FPS  32 
#define FRAME_TIME_MICROS (1000000/FPS)
#define OFFSET_MICROS 850

SDL_Window *window = NULL;
SDL_Renderer *renderer = NULL;
SDL_Texture *texture = NULL;
SDL_Event event;

uint32_t pixels[SENSOR_W * SENSOR_H];
bool running = true;

// Function to start FFmpeg for streaming
FILE* start_ffmpeg_stream() {
    const char* command =
        "ffmpeg -y -f rawvideo -pixel_format rgba -video_size 12x16 "
        "-framerate 4 -i - -f mpegts udp://10.42.0.179:1234";
    FILE* ffmpeg = popen(command, "w");
    if (!ffmpeg) {
        std::cerr << "Failed to start FFmpeg process for streaming!" << std::endl;
        exit(1);
    }
    return ffmpeg;
}
// Enhanced Heatmap function
void put_pixel_false_colour(int x, int y, double v) {
    const int NUM_COLORS = 7;
    static float color[NUM_COLORS][3] = {{0,0,1}, {0,1,1}, {0,1,0}, {1,1,0}, {1,0.5,0}, {1,0,0}, {1,0,1}};
    int idx1, idx2;
    float fractBetween = 0;
    float vmin = 5.0;
    float vmax = 50.0;
    float vrange = vmax - vmin;
    v -= vmin;
    v /= vrange;

    if (v <= 0) idx1 = idx2 = 0;
    else if (v >= 1) idx1 = idx2 = NUM_COLORS - 1;
    else {
        v *= (NUM_COLORS - 1);
        idx1 = floor(v);
        idx2 = idx1 + 1;
        fractBetween = v - float(idx1);
    }

    int ir = (int)((((color[idx2][0] - color[idx1][0]) * fractBetween) + color[idx1][0]) * 255.0);
    int ig = (int)((((color[idx2][1] - color[idx1][1]) * fractBetween) + color[idx1][1]) * 255.0);
    int ib = (int)((((color[idx2][2] - color[idx1][2]) * fractBetween) + color[idx1][2]) * 255.0);

    int offset = (y * SENSOR_W + x);
    pixels[offset] = (ib << 16) | (ig << 8) | (ir << 0);
}

int main() {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "SDL_Init() Failed: %s\n", SDL_GetError());
        return 1;
    }

    window = SDL_CreateWindow("MLX90641 Stream", 200, 200, 480, 640, SDL_WINDOW_OPENGL);
    if (!window) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "SDL_CreateWindow() Failed: %s\n", SDL_GetError());
    }

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!renderer) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "SDL_CreateRenderer() Failed: %s\n", SDL_GetError());
    }

    texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_STREAMING, SENSOR_W, SENSOR_H);
    if (!texture) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "SDL_CreateTexture() Failed: %s\n", SDL_GetError());
    }

    static uint16_t eeMLX90641[832];
    float emissivity = 1;
    uint16_t frame[834];
    static float mlx90641To[192];
    float eTa;

    paramsMLX90641 mlx90641;
    MLX90641_DumpEE(MLX_I2C_ADDR, eeMLX90641);
    MLX90641_ExtractParameters(eeMLX90641, &mlx90641);
    MLX90641_SetRefreshRate(MLX_I2C_ADDR, 0b011);  // Set FPS to 4

    FILE* ffmpeg_stream = start_ffmpeg_stream();  // Start FFmpeg for streaming

    auto frame_time = std::chrono::microseconds(FRAME_TIME_MICROS + OFFSET_MICROS);

    while (running) {
        SDL_PollEvent(&event);
        if (event.type == SDL_QUIT) running = false;

        MLX90641_GetFrameData(MLX_I2C_ADDR, frame);
        eTa = MLX90641_GetTa(frame, &mlx90641);
        MLX90641_CalculateTo(frame, &mlx90641, emissivity, eTa, mlx90641To);
        MLX90641_BadPixelsCorrection((&mlx90641)->brokenPixel, mlx90641To);

        for (int y = 0; y < SENSOR_W; y++) {
            for (int x = 0; x < SENSOR_H; x++) {
                float val = mlx90641To[SENSOR_H * (SENSOR_W - 1 - y) + x];
                put_pixel_false_colour(y, x, val);
            }
        }

        SDL_UpdateTexture(texture, NULL, (uint8_t*)pixels, SENSOR_W * sizeof(uint32_t));
        SDL_RenderCopy(renderer, texture, NULL, NULL);
        SDL_RenderPresent(renderer);

        fwrite(pixels, sizeof(uint32_t), SENSOR_W * SENSOR_H, ffmpeg_stream);  // Stream to FFmpeg
        fflush(ffmpeg_stream);

        std::this_thread::sleep_for(frame_time);
    }

    pclose(ffmpeg_stream);  // Close FFmpeg stream
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
