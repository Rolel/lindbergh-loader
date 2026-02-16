#include <SDL3/SDL_gamepad.h>
#include <SDL3/SDL_joystick.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <libudev.h>
#include <linux/input.h>
#include <pthread.h>
#include <libgen.h>

#include "wiimoteEvdev.h"

typedef struct
{
    int sdlPlayerIndex;
    char mac[18];
    int fd;
    pthread_t threadId;
    int running;
} WiiMoteDevice;

static WiiMoteDevice gWiiMotes[MAX_JOYSTICKS];
static int gWiiMoteCount = 0;
Uint32 SDL_WIIMOTION_EVENT = 0;

static void *wiimoteThreadFunc(void *arg)
{
    WiiMoteDevice *wiimote = (WiiMoteDevice *)arg;
    struct input_event ev;
    int irX = 0;
    int irY = 0;

    printf("Started IR thread for WiiMote P%d\n", wiimote->sdlPlayerIndex + 1);

    while (wiimote->running)
    {
        if (read(wiimote->fd, &ev, sizeof(ev)) > 0)
        {
            if (ev.type == EV_ABS)
            {
                if (ev.code == ABS_HAT0X)
                {
                    irX = ev.value;
                }
                else if (ev.code == ABS_HAT0Y)
                {
                    irY = ev.value;
                }
            }
            else if (ev.type == EV_SYN && ev.code == SYN_REPORT)
            {
                SDL_Event event;
                SDL_zero(event);
                event.type = SDL_WIIMOTION_EVENT;
                event.user.code = wiimote->sdlPlayerIndex;
                event.user.data1 = (void *)(intptr_t)irX;
                event.user.data2 = (void *)(intptr_t)irY;
                SDL_PushEvent(&event);
            }
        }
        usleep(1000);
    }

    printf("Exiting IR thread for WiiMote P%d\n", wiimote->sdlPlayerIndex + 1);
    close(wiimote->fd);
    return NULL;
}

static int getMacFromSyspath(struct udev *udev, const char *syspath, char *macBuffer, size_t len)
{
    int ret = -1;
    struct udev_device *dev = udev_device_new_from_syspath(udev, syspath);
    if (!dev)
    {
        return -1;
    }

    struct udev_device *parent = udev_device_get_parent_with_subsystem_devtype(dev, "hid", NULL);
    if (parent)
    {
        const char *uniq = udev_device_get_property_value(parent, "HID_UNIQ");
        if (uniq)
        {
            strncpy(macBuffer, uniq, len - 1);
            macBuffer[len - 1] = '\0';
            ret = 0;
        }
    }

    udev_device_unref(dev);
    return ret;
}

int findAndOpenWiiMotes(SDLControllers *sdlControllers)
{
    struct udev *udev = udev_new();
    if (!udev)
    {
        fprintf(stderr, "Failed to create udev context.\n");
        return 0;
    }

    for (int i = 0; i < MAX_JOYSTICKS; i++)
    {
        SDL_Joystick *joy = NULL;
        if (sdlControllers->controllers[i])
            joy = SDL_GetGamepadJoystick(sdlControllers->controllers[i]);
        else if (sdlControllers->joysticks[i])
            joy = sdlControllers->joysticks[i];

        if (joy && strstr(SDL_GetJoystickName(joy), "Nintendo Wii Remote"))
        {
            const char *sdlDevPath = SDL_GetJoystickPathForID(i);
            if (!sdlDevPath)
            {
                fprintf(stderr, "Couldn't get device path for joystick %d\n", i);
                continue;
            }

            char *devPathCopy = strdup(sdlDevPath);
            if (!devPathCopy)
                continue;

            struct udev_device *dev = udev_device_new_from_subsystem_sysname(udev, "input", basename(devPathCopy));
            free(devPathCopy);

            if (dev)
            {
                const char *syspath = udev_device_get_syspath(dev);
                if (syspath)
                {
                    if (getMacFromSyspath(udev, syspath, gWiiMotes[gWiiMoteCount].mac, sizeof(gWiiMotes[gWiiMoteCount].mac)) == 0)
                    {
                        gWiiMotes[gWiiMoteCount].sdlPlayerIndex = i;
                        printf("Found WiiMote P%d with MAC: %s\n", i + 1, gWiiMotes[gWiiMoteCount].mac);
                        gWiiMoteCount++;
                    }
                }
                udev_device_unref(dev);
            }
        }
    }

    if (gWiiMoteCount == 0)
    {
        printf("No Wii Remotes found by SDL.\n");
        udev_unref(udev);
        return 0;
    }

    struct udev_enumerate *enumerate = udev_enumerate_new(udev);
    udev_enumerate_add_match_subsystem(enumerate, "input");
    udev_enumerate_scan_devices(enumerate);

    struct udev_list_entry *devices = udev_enumerate_get_list_entry(enumerate);
    struct udev_list_entry *devListEntry;

    udev_list_entry_foreach(devListEntry, devices)
    {
        const char *path = udev_list_entry_get_name(devListEntry);
        struct udev_device *dev = udev_device_new_from_syspath(udev, path);
        if (!dev)
            continue;

        const char *devnode = udev_device_get_devnode(dev);
        if (devnode && strstr(devnode, "/dev/input/event"))
        {
            struct udev_device *parent = udev_device_get_parent_with_subsystem_devtype(dev, "input", NULL);
            if (parent)
            {
                const char *name = udev_device_get_sysattr_value(parent, "name");
                if (name && strstr(name, "Nintendo Wii Remote IR"))
                {
                    char macFromHid[18] = {0};
                                        if (getMacFromSyspath(udev, path, macFromHid, sizeof(macFromHid)) == 0)
                    {
                        for (int j = 0; j < gWiiMoteCount; j++)
                        {
                            if (strcmp(macFromHid, gWiiMotes[j].mac) == 0)
                            {
                                gWiiMotes[j].fd = open(devnode, O_RDONLY | O_NONBLOCK);
                                int flags = fcntl(gWiiMotes[j].fd, F_GETFL, 0);
                                fcntl(gWiiMotes[j].fd, F_SETFL, flags | O_NONBLOCK);
                                if (gWiiMotes[j].fd > 0)
                                {
                                    printf("Successfully associated WiiMote P%d with IR device %s\n", gWiiMotes[j].sdlPlayerIndex + 1,
                                           devnode);
                                }
                                break;
                            }
                        }
                    }
                }
            }
        }
        udev_device_unref(dev);
    }

    udev_enumerate_unref(enumerate);
    udev_unref(udev);

    return gWiiMoteCount;
}

void startWiimoteThreads()
{
    for (int i = 0; i < gWiiMoteCount; i++)
    {
        if (gWiiMotes[i].fd > 0)
        {
            gWiiMotes[i].running = 1;
            pthread_create(&gWiiMotes[i].threadId, NULL, wiimoteThreadFunc, &gWiiMotes[i]);
        }
    }
}

void cleanupWiiMoteThreads()
{
    for (int i = 0; i < gWiiMoteCount; i++)
    {
        if (gWiiMotes[i].running)
        {
            gWiiMotes[i].running = 0;
            pthread_join(gWiiMotes[i].threadId, NULL);
        }
    }
}