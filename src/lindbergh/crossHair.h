#pragma once

#include <time.h>
#include <stdbool.h>
#include <SDL3/SDL_surface.h> 
typedef struct
{
    float x, y;
    int width, height;

    unsigned int texture;
    unsigned int vao;
    unsigned int vbo;
    SDL_Surface *surface;
    time_t lastMovementTime;
    bool visible;
} Crosshair;

extern bool p1CrossHairInitialized;
extern bool p2CrossHairInitialized;

void initCrossHairs();
int loadCrosshairImage(int player, const char *filepath);
void updateCrosshairPosition(int player, float normX, float normY);
void renderCrosshairs(void);
void destroyCrosshairs(void);

void startPollingThread();
void stopPollingThread();