#pragma once

#include "sdlInput.h"

extern Uint32 SDL_WIIMOTION_EVENT;

int findAndOpenWiiMotes(SDLControllers *sdl_controllers);
void startWiimoteThreads();
void cleanupWiiMoteThreads();