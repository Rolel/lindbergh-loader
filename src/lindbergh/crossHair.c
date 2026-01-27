// #include <SDL3/SDL_timer.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#define GL_GLEXT_PROTOTYPES
#include <dlfcn.h>
#include <GL/gl.h>
#include <GL/glext.h>
#include <pthread.h>
#include <SDL3/SDL_pixels.h>
#include <SDL3_image/SDL_image.h>
#include <stdio.h>
#include <stdlib.h>

#include "config.h"
#include "crossHair.h"
#include "resolution.h"

#define MAX_PLAYERS 2
#define INACTIVITY_TIMEOUT 3

extern uint32_t gId;
extern int gGrp;
extern int gWidth;
extern int gHeight;
extern int drawableW;
extern int drawableH;

extern int phX, phY, phW, phH;
extern int phX2, phY2, phW2, phH2;

static Crosshair crossHair[MAX_PLAYERS];
static GLuint gShaderProgram = 0;
static GLint gUProjectionLoc = -1;
static GLint gUTextureLoc = -1;
static pthread_t pollingThreadId = 0;

void (*real_glDrawArrays)(GLenum, GLint, GLsizei) = NULL;

bool p1CrossHairInitialized = false;
bool p2CrossHairInitialized = false;
bool testMode = false;

int textureIdIdxAdjust = 0;


static const char *vertex_shader_source = "#version 120\n"
                                          "attribute vec2 a_pos;\n"
                                          "attribute vec2 a_tex_coord;\n"
                                          "varying vec2 v_tex_coord;\n"
                                          "uniform mat4 u_projection;\n"
                                          "void main() {\n"
                                          "    gl_Position = u_projection * vec4(a_pos.x, a_pos.y, 0.0, 1.0);\n"
                                          "    v_tex_coord = a_tex_coord;\n"
                                          "}\n";
static const char *fragment_shader_source = "#version 120\n"
                                            "varying vec2 v_tex_coord;\n"
                                            "uniform sampler2D u_texture;\n"
                                            "void main() {\n"
                                            "    gl_FragColor = texture2D(u_texture, v_tex_coord);\n"
                                            "}\n";

GLuint compileShader(GLenum type, const char *source)
{
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, NULL);
    glCompileShader(shader);
    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        char infoLog[512];
        glGetShaderInfoLog(shader, 512, NULL, infoLog);
        fprintf(stderr, "ERROR: Shader compilation failed\n%s\n", infoLog);
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

GLuint createShaderProgram()
{
    GLuint vert = compileShader(GL_VERTEX_SHADER, vertex_shader_source);
    if (vert == 0)
        return 0;
    GLuint frag = compileShader(GL_FRAGMENT_SHADER, fragment_shader_source);
    if (frag == 0)
    {
        glDeleteShader(vert);
        return 0;
    }
    GLuint program = glCreateProgram();
    glAttachShader(program, vert);
    glAttachShader(program, frag);
    glBindAttribLocation(program, 0, "a_pos");
    glBindAttribLocation(program, 1, "a_tex_coord");
    glLinkProgram(program);
    GLint success;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success)
    {
        char infoLog[512];
        glGetProgramInfoLog(program, 512, NULL, infoLog);
        fprintf(stderr, "ERROR: Shader linking failed\n%s\n", infoLog);
    }
    glDeleteShader(vert);
    glDeleteShader(frag);
    return program;
}

void createCrosshairGeometry(Crosshair *ch)
{
    float w = (float)ch->width;
    float h = (float)ch->height;
    float vertices[] = {0.0f, h, 0.0f, 1.0f, w, h,    1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                        w,    h, 1.0f, 1.0f, w, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    glGenVertexArrays(1, &ch->vao);
    glGenBuffers(1, &ch->vbo);
    glBindVertexArray(ch->vao);
    glBindBuffer(GL_ARRAY_BUFFER, ch->vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)(2 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void initCrossHairs()
{
    if (gShaderProgram == 0)
    {
        gShaderProgram = createShaderProgram();
        gUProjectionLoc = glGetUniformLocation(gShaderProgram, "u_projection");
        gUTextureLoc = glGetUniformLocation(gShaderProgram, "u_texture");
    }

    if (gShaderProgram == 0)
        return;

    if (loadCrosshairImage(0, getConfig()->p1CrossHairPath))
        p1CrossHairInitialized = true;

    if (loadCrosshairImage(1, getConfig()->p2CrossHairPath))
        p2CrossHairInitialized = true;

    if (isTestMode() || gGrp == GROUP_HOD4_TEST || gGrp == GROUP_HOD4_SP_TEST)
        testMode = true;

    if (!testMode && gId != PRIMEVAL_HUNT)
        startPollingThread();

    if (!real_glDrawArrays)
        real_glDrawArrays = dlsym(RTLD_NEXT, "glDrawArrays");

    textureIdIdxAdjust = p1CrossHairInitialized + p2CrossHairInitialized;
}

int loadCrosshairImage(int player, const char *filepath)
{
    if (player < 0 || player >= MAX_PLAYERS)
        return 0;

    if (crossHair[player].surface)
    {
        SDL_DestroySurface(crossHair[player].surface);
        crossHair[player].surface = NULL;
    }
    if (crossHair[player].texture)
    {
        glDeleteTextures(1, &crossHair[player].texture);
        crossHair[player].texture = 0;
    }
    if (crossHair[player].vao)
    {
        glDeleteVertexArrays(1, &crossHair[player].vao);
        crossHair[player].vao = 0;
    }
    if (crossHair[player].vbo)
    {
        glDeleteBuffers(1, &crossHair[player].vbo);
        crossHair[player].vbo = 0;
    }

    SDL_Surface *surface = IMG_Load(filepath);
    if (!surface)
    {
        fprintf(stderr, "Failed to load PNG for player %d: %s\n", player + 1, SDL_GetError());
        return 0;
    }
    crossHair[player].surface = SDL_ConvertSurface(surface, SDL_PIXELFORMAT_RGBA32);
    SDL_DestroySurface(surface);

    crossHair[player].x = (int)(drawableW / 2.0f);
    crossHair[player].y = (int)(drawableH / 2.0f);
    crossHair[player].visible = false;
    // crossHair[player].lastMovementTime = time(NULL);
    crossHair[player].texture = 0;
    crossHair[player].vao = 0;
    crossHair[player].vbo = 0;

    if (gId == GHOST_SQUAD_EVOLUTION)
    {
        GLuint tex;
        glGenTextures(1, &tex);
        glBindTexture(GL_TEXTURE_2D, tex);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, crossHair[player].surface->w, crossHair[player].surface->h, 0, GL_RGBA, GL_UNSIGNED_BYTE,
                     crossHair[player].surface->pixels);

        SDL_DestroySurface(crossHair[player].surface);

        crossHair[player].width = getConfig()->customCrossHairWidth;
        crossHair[player].height = getConfig()->customCrossHairHeight;
        crossHair[player].texture = tex;
    }

    return 1;
}

void updateCrosshairPosition(int player, float normX, float normY)
{
    if (player < 0 || player >= MAX_PLAYERS)
        return;
    crossHair[player].x = normX * drawableW;
    crossHair[player].y = normY * drawableH;
    // crossHair[player].lastMovementTime = time(NULL);
    // crossHair[player].visible = true;
    if (testMode || gId == PRIMEVAL_HUNT)
        crossHair[player].visible = true;
}

void renderCrosshairs(void)
{
    if (gId == PRIMEVAL_HUNT)
        glViewport(phX, phY, phW, phH);
    else if (gGrp == GROUP_HOD4_SP)
        glViewport(0, 0, gWidth, gHeight);

    GLint texFormat = 0;
    glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_INTERNAL_FORMAT, &texFormat);

    if (texFormat == 0x1908 && gId == GHOST_SQUAD_EVOLUTION)
        return;

    GLint last_program, last_vao, last_vbo, last_active_texture, last_depth_func;
    GLboolean last_blend_enabled, last_depth_enabled; //, last_srgb_enabled;
    GLint last_blend_src, last_blend_dst;
    glGetIntegerv(GL_CURRENT_PROGRAM, &last_program);
    glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &last_vao);
    glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &last_vbo);
    glGetIntegerv(GL_ACTIVE_TEXTURE, &last_active_texture);
    last_blend_enabled = glIsEnabled(GL_BLEND);
    last_depth_enabled = glIsEnabled(GL_DEPTH_TEST);
    // last_srgb_enabled = glIsEnabled(GL_FRAMEBUFFER_SRGB);
    glGetIntegerv(GL_BLEND_SRC_RGB, &last_blend_src);
    glGetIntegerv(GL_BLEND_DST_RGB, &last_blend_dst);
    glGetIntegerv(GL_DEPTH_FUNC, &last_depth_func);

    // if (last_srgb_enabled)
    //     glDisable(GL_FRAMEBUFFER_SRGB);
    glDisable(GL_DEPTH_TEST);
    glDepthFunc(GL_ALWAYS);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glUseProgram(gShaderProgram);
    glActiveTexture(GL_TEXTURE0);
    glUniform1i(gUTextureLoc, 0);

    time_t currentTime = time(NULL);
    for (int i = 0; i < MAX_PLAYERS; ++i)
    {
        // if (difftime(currentTime, crossHair[i].lastMovementTime) > INACTIVITY_TIMEOUT)
        // {
        //     crossHair[i].visible = false;
        // }
        if (!crossHair[i].visible)
            continue;

        if (crossHair[i].texture == 0 && crossHair[i].surface != NULL)
        {
            glGenTextures(1, &crossHair[i].texture);
            glBindTexture(GL_TEXTURE_2D, crossHair[i].texture);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, crossHair[i].surface->w, crossHair[i].surface->h, 0, GL_RGBA, GL_UNSIGNED_BYTE,
                         crossHair[i].surface->pixels);

            crossHair[i].width = getConfig()->customCrossHairWidth;
            crossHair[i].height = getConfig()->customCrossHairHeight;
            createCrosshairGeometry(&crossHair[i]);

            SDL_DestroySurface(crossHair[i].surface);
            crossHair[i].surface = NULL;
        }

        if (crossHair[i].texture != 0 && crossHair[i].vao != 0)
        {
            float x = crossHair[i].x - (crossHair[i].width / 2.0f);
            float y = crossHair[i].y - (crossHair[i].height / 2.0f);
            float projection[16] = {0.0f};
            projection[0] = 2.0f / drawableW;
            projection[5] = -2.0f / drawableH;
            projection[10] = -1.0f;
            projection[12] = -1.0f + (x * projection[0]);
            projection[13] = 1.0f + (y * projection[5]);
            projection[15] = 1.0f;
            glUniformMatrix4fv(gUProjectionLoc, 1, GL_FALSE, projection);

            glBindTexture(GL_TEXTURE_2D, crossHair[i].texture);
            glBindVertexArray(crossHair[i].vao);
            real_glDrawArrays(GL_TRIANGLES, 0, 6);
        }
    }

    glBindVertexArray(last_vao);
    glBindBuffer(GL_ARRAY_BUFFER, last_vbo);
    glUseProgram(last_program);
    glActiveTexture(last_active_texture);
    glBlendFunc(last_blend_src, last_blend_dst);
    if (last_blend_enabled)
        glEnable(GL_BLEND);
    else
        glDisable(GL_BLEND);
    if (last_depth_enabled)
        glEnable(GL_DEPTH_TEST);
    else
        glDisable(GL_DEPTH_TEST);
    glDepthFunc(last_depth_func);
    // if (last_srgb_enabled)
    //     glEnable(GL_FRAMEBUFFER_SRGB);
}

void bindAndPosition(Crosshair *c)
{
    if (!c->texture)
        return;
    ;

    float x = c->x - c->width / 2.0f;
    float y = c->y - c->height / 2.0f;

    glBindTexture(GL_TEXTURE_2D, c->texture);
    glColor4f(1, 1, 1, 1);
    glBegin(GL_QUADS);
    glTexCoord2f(0, 0);
    glVertex2f(x, y);
    glTexCoord2f(1, 0);
    glVertex2f(x + c->width, y);
    glTexCoord2f(1, 1);
    glVertex2f(x + c->width, y + c->height);
    glTexCoord2f(0, 1);
    glVertex2f(x, y + c->height);
    glEnd();
}

void renderGsEvoCrosshairs(void)
{
    GLint texFormat = 0;
    glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_INTERNAL_FORMAT, &texFormat);

    if (texFormat == 0x1908 && gId == GHOST_SQUAD_EVOLUTION)
        return;

    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glDisable(GL_DEPTH_TEST); // for HOD4
    glDepthMask(GL_FALSE);
    glEnable(GL_TEXTURE_2D);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, drawableW, drawableH, 0, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    if (p1CrossHairInitialized && crossHair[0].visible)
        bindAndPosition(&crossHair[0]);

    if (p2CrossHairInitialized && crossHair[1].visible)
        bindAndPosition(&crossHair[1]);

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glDisable(GL_BLEND);
    glDisable(GL_TEXTURE_2D);
    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
    glPopAttrib();
}

void destroyCrosshairs(void)
{
    stopPollingThread();
    for (int i = 0; i < MAX_PLAYERS; i++)
    {
        if (crossHair[i].texture)
            glDeleteTextures(1, &crossHair[i].texture);
        if (crossHair[i].vao)
            glDeleteVertexArrays(1, &crossHair[i].vao);
        if (crossHair[i].vbo)
            glDeleteBuffers(1, &crossHair[i].vbo);
        if (crossHair[i].surface)
            SDL_DestroySurface(crossHair[i].surface);
    }
    if (gShaderProgram)
        glDeleteProgram(gShaderProgram);
}

void glDrawArrays(GLenum mode, GLint first, GLsizei count)
{
    if (gId == GHOST_SQUAD_EVOLUTION)
    {
        GLint currentFBO = 0;
        glGetIntegerv(GL_FRAMEBUFFER_BINDING_EXT, &currentFBO);
        renderGsEvoCrosshairs();
    }
    if (!real_glDrawArrays)
        real_glDrawArrays = dlsym(RTLD_NEXT, "glDrawArrays");
    real_glDrawArrays(mode, first, count);
}

typedef struct
{
    bool keepRunning;
} PollingArgs;

PollingArgs gPollingArgs;

static void *gsevoPollingThreadFunc(void *arg)
{
    PollingArgs *args = (PollingArgs *)arg;

    while (args->keepRunning)
    {
        uint8_t p1Mode = *(uint8_t *)0x086617E8;
        uint8_t p2Mode = *(uint8_t *)0x08661994;
        if (p1Mode == 0x2)
            crossHair[0].visible = true;
        else
            crossHair[0].visible = false;

        if (p2Mode == 0x2)
            crossHair[1].visible = true;
        else
            crossHair[1].visible = false;

        SDL_Delay(10);
    }
    return NULL;
}

static void *pollingThreadFunc(void *arg)
{
    PollingArgs *args = (PollingArgs *)arg;
    uint32_t *pGunMgr;
    uint32_t *pPlayerMgr;

    if (gId == THE_HOUSE_OF_THE_DEAD_4_REVA)
    {
        pGunMgr = (uint32_t *)0x0a711758;
        pPlayerMgr = (uint32_t *)0x0a7117a8;
    }
    else if (gId == THE_HOUSE_OF_THE_DEAD_4_REVB || gId == THE_HOUSE_OF_THE_DEAD_4_REVC)
    {
        pGunMgr = (uint32_t *)0x0a6f27a8;
        pPlayerMgr = (uint32_t *)0x0a6f27f8;
    }
    else if (gId == RAMBO)
        pPlayerMgr = (uint32_t *)0x0842fe9c;
    else if (gId == RAMBO_CHINA)
        pPlayerMgr = (uint32_t *)0x084304fc;
    else if (gId == THE_HOUSE_OF_THE_DEAD_4_SPECIAL_REVB)
        pPlayerMgr = (uint32_t *)0x0A69F92C;
    else
        args->keepRunning = false;

    while (args->keepRunning)
    {
        if (gGrp != GROUP_HOD4 || *pGunMgr != 0x0)
        {
            uint8_t *gameMode;
            if (gGrp == GROUP_HOD4)
            {
                uint32_t *gameModeAddress = *(void **)pGunMgr + 0x2c;
                gameMode = *(uint8_t **)gameModeAddress + 0x38;
            }
            if (gGrp != GROUP_HOD4 || *gameMode == 8)
            {
                if (*pPlayerMgr != 0x0)
                {
                    uint32_t *p1ModeAddress = *(void **)pPlayerMgr + 0x34;
                    uint32_t *p2ModeAddress = *(void **)pPlayerMgr + 0x38;

                    if (*p1ModeAddress != 0x0)
                    {
                        uint8_t *p1Mode = *(uint8_t **)p1ModeAddress + 0x38;
                        if (*p1Mode == 3 || *p1Mode == 5)
                            crossHair[0].visible = true;
                        else
                            crossHair[0].visible = false;
                    }

                    if (*p2ModeAddress != 0x0)
                    {
                        uint8_t *p2Mode = *(uint8_t **)p2ModeAddress + 0x38;
                        if (*p2Mode == 3 || *p2Mode == 5)
                            crossHair[1].visible = true;
                        else
                            crossHair[1].visible = false;
                    }
                }
            }
        }
        SDL_Delay(10);
    }
    return NULL;
}

void startPollingThread()
{
    if (gPollingArgs.keepRunning)
        return;

    gPollingArgs.keepRunning = true;
    if (gId == GHOST_SQUAD_EVOLUTION)
    {
        pthread_create(&pollingThreadId, NULL, gsevoPollingThreadFunc, &gPollingArgs);
    }
    else
    {
        pthread_create(&pollingThreadId, NULL, pollingThreadFunc, &gPollingArgs);
    }
}

void stopPollingThread()
{
    if (gPollingArgs.keepRunning)
    {
        gPollingArgs.keepRunning = false;
        if (pollingThreadId)
        {
            pthread_join(pollingThreadId, NULL);
            pollingThreadId = 0;
        }
    }
}
