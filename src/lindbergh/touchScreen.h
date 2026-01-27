#pragma once

#include <SDL3/SDL_events.h>
#include <stdint.h>
#include <sys/types.h>

void phCoordinates(int x, int y, int w, int h, int button);
ssize_t phRead(int fd, void *buf, size_t count);
void phTouchScreenCursor(int mX, int mY, int *motX, int *motY);
bool phIsInsideTouchScreen(int mX, int mY, int *x, int *y);
void phTouchClick(int x, int y, int type);
void handleMahjongTouch(const SDL_Event *e, int drawableW, int drawableH);
void mj4BuildResponsePacket(uint8_t cmd, int pckA, int pckB, int pckC);
int mj4ReadTouchPacket(uint8_t *buf, size_t max_len);
int mj4WriteTouchPacket(const void *buf, size_t count);
void mj4TouchHolding();