#include <SDL3/SDL_events.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "customCursor.h"
#include "config.h"
#include "touchScreen.h"

char phSendBuf[10];
bool phBufferFilled = false;
int phCommand = 1;
int phCommandByte = 0;
uint16_t coordX;
uint16_t coordY;
extern int phX, phY, phW, phH;
extern int phX2, phY2, phW2, phH2;
extern void *customCursor;
extern void *touchCursor;
extern bool phShowCursorInGame;

#define TOUCH_START 0x81
#define TOUCH_TOUCHING 0x82
#define TOUCH_END 0x84

#define MJ4_PACKET_SIZE 10
uint8_t mj4ResponseBuf[MJ4_PACKET_SIZE] = {0};
bool mj4MousePressed = false;
extern int mj4MouseX;
extern int mj4MouseY;
bool mj4ResponseReady = false;
bool mj4TouchedInsideScreen = false;

void phCoordinates(int x, int y, int w, int h, int button)
{
    coordX = (float)x / ((float)w * 0.0002441406);
    coordY = (float)y / ((float)h * 0.0002441406);
    memset(phSendBuf, 0, sizeof(phSendBuf));

    if (button == SDL_EVENT_MOUSE_BUTTON_DOWN)
    {
        phSendBuf[2] = TOUCH_START;
        phCommand = 1;
    }
    else if (button == SDL_EVENT_MOUSE_BUTTON_UP)
    {
        phSendBuf[2] = TOUCH_END;
        phCommand = 3;
    }
    else if (button == SDL_EVENT_MOUSE_MOTION)
    {
        phSendBuf[2] = TOUCH_TOUCHING;
        phCommand = 2;
    }
    phSendBuf[0] = 0x55;
    phSendBuf[1] = 0x54;
    phSendBuf[3] = (char)(coordX & 0xFF);
    phSendBuf[4] = (char)((coordX >> 8) & 0xff);
    phSendBuf[5] = (char)(coordY & 0xFF);
    phSendBuf[6] = (char)((coordY >> 8) & 0xff);

    phBufferFilled = true;
}

uint8_t phCalcChecksum()
{
    int8_t checksum = -0x56;
    for (int x = 0; x < 9; x++)
    {
        checksum += phSendBuf[x];
    }
    return (uint8_t)(checksum & 0xff);
}

ssize_t phRead(int fd, void *buf, size_t count)
{
    if (!phBufferFilled)
        return 0;

    if (phCommandByte == 9)
    {
        switch (phCommand)
        {
            case 1:
                phCommand = 2;
                break;
            case 2:
                phSendBuf[2] = TOUCH_TOUCHING;
                break;
            case 3:
                phBufferFilled = false;
        }
        phSendBuf[9] = phCalcChecksum();
    }
    memcpy(buf, &phSendBuf[phCommandByte], sizeof(char));
    phCommandByte++;
    if (phCommandByte == 10)
        phCommandByte = 0;
    return 1;
}

void phTouchScreenCursor(int mX, int mY, int *motX, int *motY)
{
    if (getConfig()->emulateTouchscreen)
    {
        switch (getConfig()->phScreenMode)
        {
            case 0:
            case 2:
            case 3:
                if ((mX >= phX2 && mX <= (phX2 + phW2) && mY >= phY2 && mY <= (phY2 + phH2)) && getConfig()->hideCursor == 0)
                {
                    setCursor(touchCursor);
                    showPhCursor();
                }
                else if (getConfig()->hideCursor == 1 || phShowCursorInGame == false)
                {
                    hideCursor();
                }
                else if (phShowCursorInGame && getConfig()->hideCursor == 0)
                {
                    setCursor(customCursor);
                    showCursor();
                }
                break;
            case 4:
                if ((mX >= phX2 && mX <= (phX2 + phW2) && mY >= (phY2 + phH) && mY <= (phY2 + (phH2 + phH))) &&
                    getConfig()->hideCursor == 0)
                {
                    setCursor(touchCursor);
                    showPhCursor();
                }
                else if (getConfig()->hideCursor == 1 || phShowCursorInGame == false)
                {
                    hideCursor();
                }
                else if (phShowCursorInGame && getConfig()->hideCursor == 0)
                {
                    setCursor(customCursor);
                    showCursor();
                }
        }
    }
    if (mX < phX)
        *motX = phX;
    if (mX > (phW + phX))
        *motX = (phW + phX) - 1;
    if (mY > phH + phY)
        *motY = (phH + phY) - 1;
    if (mY < phY)
        *motY = phY;
}

bool phIsInsideTouchScreen(int mX, int mY, int *x, int *y)
{
    switch (getConfig()->phScreenMode)
    {
        case 0:
        case 2:
        case 3:
            if (mX >= phX2 && mX <= (phX2 + phW2) && mY >= phY2 && mY <= (phY2 + phH2))
            {
                *x = mX - phW;
                *y = mY - phY2;
                return true;
            }
            break;
        case 4:
            if (mX >= phX2 && mX <= (phX2 + phW2) && mY >= (phY2 + phH) && mY <= (phY2 + (phH2 + phH)))
            {
                *x = mX - phX2;
                *y = mY - phH;
                return true;
            }
    }
    return false;
}

void phTouchClick(int x, int y, int type)
{   
    if (type == SDL_EVENT_MOUSE_BUTTON_DOWN)
        phCoordinates(x, y, phW2, phH2, SDL_EVENT_MOUSE_BUTTON_DOWN);
    else if (type == SDL_EVENT_MOUSE_BUTTON_UP)
        phCoordinates(x, y, phW2, phH2, SDL_EVENT_MOUSE_BUTTON_UP);
    else if (type == SDL_EVENT_MOUSE_MOTION)
        phCoordinates(x, y, phW2, phH2, SDL_EVENT_MOUSE_MOTION);
}

void handleMahjongTouch(const SDL_Event *e, int drawableW, int drawableH)
{

    mj4MousePressed = 0;
    mj4MouseX = e->button.x;
    mj4MouseY = drawableH - e->button.y;

    if (e->type == SDL_EVENT_MOUSE_BUTTON_DOWN)
        mj4MousePressed = 1;

    float winAspect = (float)drawableW / drawableH;
    float gameAspect = 1024.0f / 768.0f;

    int viewportX, viewportY, viewportW, viewportH;

    if (winAspect > gameAspect)
    {
        viewportH = drawableH;
        viewportW = (int)(drawableH * gameAspect);
        viewportX = (drawableW - viewportW) / 2;
        viewportY = 0;
    }
    else
    {
        viewportW = drawableW;
        viewportH = (int)(drawableW / gameAspect);
        viewportX = 0;
        viewportY = (drawableH - viewportH) / 2;
    }

    if (mj4MouseX < viewportX || mj4MouseX >= viewportX + viewportW || mj4MouseY < viewportY || mj4MouseY >= viewportY + viewportH)
    {
        mj4TouchedInsideScreen = false;
        return;
    }
    else
    {
        mj4TouchedInsideScreen = true;
        float normalizedX = (mj4MouseX - viewportX) / (float)viewportW;
        float normalizedY = (mj4MouseY - viewportY) / (float)viewportH;

        mj4MouseX = normalizedX * 1024.0f;
        mj4MouseY = normalizedY * 768.0f;
    }
    mj4BuildResponsePacket(0x54, mj4MouseX, mj4MouseY, mj4MousePressed);
}

uint8_t mj4CalcChecksum(uint8_t *packet)
{
    uint8_t sum = 0;
    for (int i = 0; i < MJ4_PACKET_SIZE - 1; ++i)
        sum += packet[i];
    return (sum + 0xAA) & 0xFF;
}

void mj4BuildResponsePacket(uint8_t cmd, int pckA, int pckB, int pckC)
{
    uint8_t *packet = mj4ResponseBuf;

    packet[0] = 0x55;
    packet[1] = cmd;

    if (cmd == 0x54)
    {
        packet[2] = pckC; // & 0xFF;
        packet[3] = pckA & 0xFF;
        packet[4] = (pckA >> 8) & 0xFF;
        packet[5] = pckB & 0xFF;
        packet[6] = (pckB >> 8) & 0xFF;
    }
    else
    {
        packet[2] = pckA & 0xFF;
        packet[3] = pckB & 0xFF;
        packet[4] = pckC & 0xFF;
        packet[5] = 0x00;
        packet[6] = 0x00;
    }
    packet[7] = 0x00;
    packet[8] = 0x00;
    packet[9] = mj4CalcChecksum(packet);

    mj4ResponseReady = true;
}

int mj4WriteTouchPacket(const void *buf, size_t count)
{
    static uint8_t packetBuf[10];
    static int packetPos = 0;
    bool degugMsg = getConfig()->showDebugMessages;

    for (size_t i = 0; i < count; ++i)
    {
        uint8_t b = ((uint8_t *)buf)[i];

        if (packetPos == 0 && b != 0x55)
            continue;

        packetBuf[packetPos++] = b;

        if (packetPos == MJ4_PACKET_SIZE)
        {
            if (degugMsg)
            {
                printf("Serial1 FULL PACKET: ");
                for (int j = 0; j < MJ4_PACKET_SIZE; ++j)
                    printf("%02X ", packetBuf[j]);
                printf("\n");
            }

            uint8_t cmd = packetBuf[1];

            switch (cmd)
            {
                case 0x52:
                    if (degugMsg)
                        printf("MJ4: Init/handshake? (0x52)\n");
                    // mj4_send_response(0x52, 0xff, 0xff);
                    // start_streaming = 1;
                    break;
                case 0x50:
                    if (degugMsg)
                        printf("MJ4: Config/setup (baud) (0x50)\n");
                    // mj4_send_response(0x50, 0x00, 0x00);
                    break;
                case 0x70:
                    if (degugMsg)
                        printf("MJ4: Status check/reply? (0x70)\n");
                    mj4BuildResponsePacket(0x50, 0x30, 0x06, 0x04);
                    break;
                case 0x42:
                    if (degugMsg)
                        printf(" (0x42)\n");
                    mj4BuildResponsePacket(0x41, 0x03, 0x01, 0x00);
                    break;
                case 0x62:
                    if (degugMsg)
                        printf(" (0x62)\n");
                    mj4BuildResponsePacket(0x42, 0x03, 0x01, 0x00);
                    break;
                case 0x6D:
                    if (degugMsg)
                        printf(" (0x6D)\n");
                    mj4BuildResponsePacket(0x4D, 0x00, 0x00, 0x00);
                    break;
                case 0x4D:
                    if (degugMsg)
                        printf(" (0x6D)\n");
                    mj4BuildResponsePacket(0x41, 0x03, 0x01, 0x00);
                    break;
                default:
                    if (degugMsg)
                        printf("MJ4: Unknown command %02X\n", cmd);
                    mj4ResponseReady = false;
                    break;
            }
            packetPos = 0;
        }
    }
    return count;
}

int mj4ReadTouchPacket(uint8_t *buf, size_t count)
{
    if (mj4ResponseReady && count >= MJ4_PACKET_SIZE)
    {
        memcpy(buf, mj4ResponseBuf, MJ4_PACKET_SIZE);
        mj4ResponseReady = false;
        return MJ4_PACKET_SIZE;
    }
    return 0;
}

void mj4TouchHolding()
{
    if (mj4MousePressed && mj4TouchedInsideScreen)
        mj4BuildResponsePacket(0x54, mj4MouseX, mj4MouseY, 2);
}