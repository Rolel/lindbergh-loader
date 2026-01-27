#include <SDL3/SDL.h>
#include <SDL3/SDL_haptic.h>
#include <stdio.h>
#include <string.h>

#include "forceFeedback.h"
#include "sdlInput.h"

static int effect_id = -1;
extern SDLControllers sdlJoysticks;

void sdlFfbInit(void)
{
    int i, num_haptics;
    SDL_HapticID *haptics = SDL_GetHaptics(&num_haptics);
    if (haptics)
    {
        for (i = 0; i < num_haptics; ++i)
        {
            SDL_HapticID instance_id = haptics[i];
            SDL_Log("Haptic %" SDL_PRIu32 ": %s", instance_id, SDL_GetHapticNameForID(instance_id));
            sdlJoysticks.haptics[i] = SDL_OpenHaptic(instance_id);
            sdlFfbRumble(1.0, 1.0, 200);
        }

        SDL_free(haptics);
    }
    else {
        SDL_Log("No haptic support!");
    }
    
    // int num_joysticks;
    // SDL_JoystickID *joystics = SDL_GetJoysticks(&num_joysticks);
    // if (joystics)
    // {
    //     SDL_JoystickID instance_id;
    //     for (int i = 0; i < num_joysticks; ++i)
    //     {
    //         instance_id = joystics[i];
    //         // sdlJoysticks.haptics[i] = SDL_HapticOpenFromJoystick(sdlJoysticks.joysticks[i]);
    //         // sdlJoysticks.haptics[i] = SDL_OpenHapticFromJoystick(SDL_GetGamepadJoystick(sdlJoysticks.controllers[i]));
    //         sdlJoysticks.haptics[i] = SDL_OpenHaptic(instance_id);
    //         printf("JoyName: %s\n", SDL_GetJoystickNameForID(instance_id));
    //         if (sdlJoysticks.haptics[i] && SDL_GetHapticFeatures(sdlJoysticks.haptics[i]) & SDL_HAPTIC_LEFTRIGHT)
    //         {
    //             printf("Haptic initialized!\n");
    //             sdlFfbRumble(1.0, 1.0, 200);
    //         }
    //         else
    //         {
    //             printf("No haptic support!\n");
    //             sdlJoysticks.haptics[i] = NULL;
    //         }
    //     }
    // }
}

void sdlFfbRumble(float left, float right, int duration_ms)
{
    if (!sdlJoysticks.haptics[0])
        return;
    SDL_HapticEffect effect;
    memset(&effect, 0, sizeof(effect));
    effect.type = SDL_HAPTIC_LEFTRIGHT;
    effect.leftright.length = duration_ms;
    effect.leftright.large_magnitude = (Uint16)(left * 0xFFFF);
    effect.leftright.small_magnitude = (Uint16)(right * 0xFFFF);
    if (effect_id >= 0)
        SDL_DestroyHapticEffect(sdlJoysticks.haptics[0], effect_id);
    effect_id = SDL_CreateHapticEffect(sdlJoysticks.haptics[0], &effect);
    SDL_RunHapticEffect(sdlJoysticks.haptics[0], effect_id, 1);
}

void sdlFfbShutdown(void)
{
    if (sdlJoysticks.haptics[0])
    {
        SDL_CloseHaptic(sdlJoysticks.haptics[0]);
        sdlJoysticks.haptics[0] = NULL;
    }
}

void sdlFfbDriveboard(const unsigned char *buffer, size_t count)
{

    // printf("FFB driveboard: count=%zu, data:", count);
    // for (size_t i = 0; i < count; ++i) {
    //     printf(" 0x%02x", buffer[i]);
    // }
    // printf("\n");

    switch (buffer[0])
    {
        case 0x80:
        { // power
            if (buffer[2] == 0x01)
            {
                printf("0x80 command: Triggering full-power rumble\n");
                // ffb_rumble(1.0f, 1.0f, 500);
            }
            break;
        }
        case 0x85:
        { // Rumble/vibrate
            uint8_t speed = buffer[1];
            uint8_t power = buffer[2];

            float force = (float)power / 63.0f; // Map power 1-63 to 0.0-1.0

            int duration = 100;
            printf("Triggering rumble: force=%.2f, duration=%dms\n", force, duration);
            sdlFfbRumble(force, force, duration);
            break;
        }
        case 0x84:
        { // Movement command
            uint8_t direction = buffer[1];
            uint8_t value = buffer[2];

            float left = 0.0f, right = 0.0f;
            int duration = 100; // ms

            if (direction == 0x00)
            { // right - value from 0x7F (min) to 0x01 (max)

                right = (127.0f - (float)value) / 127.0f;
            }
            else if (direction == 0x01)
            { // left  - value goes from 0x00 (min) to 0x7F (max)

                left = (float)value / 127.0f;
            }

            printf("Triggering movement rumble: left=%.2f, right=%.2f, duration=%dms\n", left, right, duration);
            // sdlFfbRumble(left, right, duration);
            break;
        }
        case 0x86: // Friction
            printf("Friction:");
            printf(" Power: %d", buffer[1]);
            printf(" / Percentage: %d\n", buffer[2]);
            break;
        case 0x8:
            break;
        case 0x87: // move and set target point
            printf("Move and set target point\n");
            break;
        case 0x88: // move to current target point
            printf("Move to current target point\n");
            break;
        case 0x8B: // centering strength
            printf("Centering strength:");
            printf(" param1: %d", buffer[1]);
            printf(" / param2: %d\n", buffer[2]);
            break;
        case 0xFB:
        { // Playback sud package

            uint8_t effect = buffer[2];
            printf("Outrun FFB 0xFB effect: 0x%02x\n", effect);

            if (effect == 0x02)
            {
                sdlFfbRumble(1.0f, 1.0f, 120);
            }
            else if (effect == 0x10 || effect == 0x0B || effect == 0x04)
            {
                sdlFfbRumble(0.0f, 1.0f, 120);
            }
            else if (effect == 0x00 || effect == 0x1B || effect == 0x14)
            {
                sdlFfbRumble(1.0f, 0.0f, 120);
            }
            break;
        }
        default:
            break;
    }
}

void ffb_output(const unsigned char *buffer, size_t count)
{
    /*     printf("FFB output: count=%zu, data:", count);
        for (size_t i = 0; i < count; ++i)
            printf(" 0x%02x", buffer[i]);
        printf("\n"); */

    if (count > 0)
    {
        if (buffer[0] & 0x40)
        {
            // printf("GPO: ABC Vibration triggered (bit 6 set)\n");
            // sdlFfbRumble(1.0f, 1.0f, 120);
        }
    }
}