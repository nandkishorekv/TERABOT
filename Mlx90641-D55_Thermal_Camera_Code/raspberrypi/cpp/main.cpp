#include <stdint.h>
#include <iostream>
#include <cstring>
#include <fstream>
#include <chrono>
#include <thread>
#include <math.h>
#include "MLX90641_API.h"
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <SDL2/SDL_render.h>

#define MLX_I2C_ADDR 0x33
#define SENSOR_W 12
#define SENSOR_H 16
#define FPS 16
#define FRAME_TIME_MICROS (1000000 / FPS)
#define OFFSET_MICROS 850

SDL_Window *window = nullptr;
SDL_Renderer *renderer = nullptr;
SDL_Texture *texture = nullptr;
SDL_Event event;

uint32_t pixels[SENSOR_W * SENSOR_H];
bool running = true;

// Heatmap function
void put_pixel_false_colour(int x, int y, double v) {
    const int NUM_COLORS = 7;
    static float color[NUM_COLORS][3] = {{0, 0, 0}, {0, 0, 1}, {0, 1, 0}, {1, 1, 0}, {1, 0, 0}, {1, 0, 1}, {1, 1, 1}};
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

// Bilinear interpolation for upscaling
uint32_t bilinear_interpolate(uint32_t *src, int src_w, int src_h, float x, float y) {
    int x1 = floor(x);
    int y1 = floor(y);
    int x2 = x1 + 1;
    int y2 = y1 + 1;

    if (x2 >= src_w) x2 = src_w - 1;
    if (y2 >= src_h) y2 = src_h - 1;

    uint32_t q11 = src[y1 * src_w + x1];
    uint32_t q21 = src[y1 * src_w + x2];
    uint32_t q12 = src[y2 * src_w + x1];
    uint32_t q22 = src[y2 * src_w + x2];

    float dx = x - x1;
    float dy = y - y1;

    uint32_t r1 = (q11 & 0xFF) * (1 - dx) + (q21 & 0xFF) * dx;
    uint32_t g1 = ((q11 >> 8) & 0xFF) * (1 - dx) + ((q21 >> 8) & 0xFF) * dx;
    uint32_t b1 = ((q11 >> 16) & 0xFF) * (1 - dx) + ((q21 >> 16) & 0xFF) * dx;

    uint32_t r2 = (q12 & 0xFF) * (1 - dx) + (q22 & 0xFF) * dx;
    uint32_t g2 = ((q12 >> 8) & 0xFF) * (1 - dx) + ((q22 >> 8) & 0xFF) * dx;
    uint32_t b2 = ((q12 >> 16) & 0xFF) * (1 - dx) + ((q22 >> 16) & 0xFF) * dx;

    uint32_t r = r1 * (1 - dy) + r2 * dy;
    uint32_t g = g1 * (1 - dy) + g2 * dy;
    uint32_t b = b1 * (1 - dy) + b2 * dy;

    return (b << 16) | (g << 8) | r;
}
FILE* start_ffmpeg_stream() {
    const char* command =
        "ffmpeg -y -f rawvideo -pixel_format rgba -video_size 192x256 "  // Upscaled resolution
        "-framerate 4 -i - -f mpegts udp://10.42.0.143:1234";
    FILE* ffmpeg = popen(command, "w");
    if (!ffmpeg) {
        std::cerr << "Failed to start FFmpeg process for streaming!" << std::endl;
        exit(1);
    }
    return ffmpeg;
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

        // Upscale the image to 192x256 using bilinear interpolation
        uint32_t upscaled_pixels[192 * 256];
        for (int y = 0; y < 256; y++) {
            for (int x = 0; x < 192; x++) {
                float src_x = (float)x / 192 * SENSOR_W;
                float src_y = (float)y / 256 * SENSOR_H;
                upscaled_pixels[y * 192 + x] = bilinear_interpolate(pixels, SENSOR_W, SENSOR_H, src_x, src_y);
            }
        }

        // Stream the upscaled image
        fwrite(upscaled_pixels, sizeof(uint32_t), 192 * 256, ffmpeg_stream);
        fflush(ffmpeg_stream);

        std::this_thread::sleep_for(frame_time);
    }

    pclose(ffmpeg_stream);  // Close FFmpeg stream
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
