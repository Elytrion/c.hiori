

/*---------------------------------------------------------
 * file:	main.c
 * author:	[NAME]
 * email:	[DIGIPEN EMAIL ADDRESS]
*
 * brief:	Main entry point for the sample project
            * of the CProcessing library
*
 * documentation link:
 * https://inside.digipen.edu/main/GSDP:GAM100/CProcessing
*
 * Copyright © 2020 DigiPen, All rights reserved.
* ---------------------------------------------------------*/

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "cprocessing.h"




typedef struct _Particle
{
    CP_Vector pos;
    CP_Vector vel;
    CP_Color* color;
} Particle;

const float EPSILON = 0.0000001f;

float particleSize = 3.0f;

CP_Color randomColors[] = {
    { 128, 0,   0,   255 },
    { 128, 128, 0,   255 },
    { 0,   128, 0,   255 },
    { 0,   128, 128, 255 },
    { 0,   0,   128, 255 },
    { 128, 0,   128, 255 } };

void ParticleCreate(Particle* part) {
    int canvasWidth = CP_System_GetWindowWidth();
    int canvasHeight = CP_System_GetWindowHeight();
    part->pos.x = CP_Random_RangeFloat(0, (float)canvasWidth);
    part->pos.y = CP_Random_RangeFloat(0, (float)canvasHeight);
    part->vel.x = CP_Random_RangeFloat(-150, 150);
    part->vel.y = CP_Random_RangeFloat(-150, 150);
    part->color = &randomColors[CP_Random_RangeInt(0, 5)];
}

void ParticleDisplay(Particle* part)
{
    CP_Graphics_DrawEllipse(part->pos.x, part->pos.y, particleSize, particleSize);
}

void ParticleUpdate(Particle* part)
{
    int canvasWidth = CP_System_GetWindowWidth();
    int canvasHeight = CP_System_GetWindowHeight();

    // move particle based on velocity and correct for wall collisions
    float time = CP_System_GetDt();
    float timeX = time;
    float timeY = time;

    while (time > EPSILON)
    {
        bool collisionX = false;
        bool collisionY = false;

        float newPosX = part->pos.x + part->vel.x * time;
        float newPosY = part->pos.y + part->vel.y * time;
        float newTime = time;

        // check wall collisions X and Y
        if (newPosX <= 0)
        {
            timeX = part->pos.x / (part->pos.x - newPosX) * time;
            collisionX = true;
        }
        else if (newPosX >= canvasWidth)
        {
            timeX = (canvasWidth - part->pos.x) / (newPosX - part->pos.x) * time;
            collisionX = true;
        }

        if (newPosY <= 0)
        {
            timeY = part->pos.y / (part->pos.y - newPosY) * time;
            collisionY = true;
        }
        else if (newPosY >= canvasHeight)
        {
            timeY = (canvasHeight - part->pos.y) / (newPosY - part->pos.y) * time;
            collisionY = true;
        }

        // resolve collisions
        if ((collisionX == true) || (collisionY == true))
        {

            // take the nearest time
            if (timeX < timeY)
            {
                newTime = timeX;
            }
            else
            {
                newTime = timeY;
            }

            // move the particle
            part->pos.x += part->vel.x * newTime;
            part->pos.y += part->vel.y * newTime;

            // flip velocity vectors to reflect off walls
            if ((collisionX == true) && (collisionY == false))
            {
                part->vel.x *= -1;
            }
            else if ((collisionX == false) && (collisionY == true))
            {
                part->vel.y *= -1;
            }
            else
            {	// they must both be colliding for this condition to occur
                if (timeX < timeY)
                {
                    part->vel.x *= -1;
                }
                else if (timeX > timeY)
                {
                    part->vel.y *= -1;
                }
                else
                {	// they must be colliding at the same time (ie. a corner)
                    part->vel.x *= -1;
                    part->vel.y *= -1;
                }
            }

            // decrease time and iterate
            time -= newTime;
        }
        else
        {
            // no collision
            part->pos.x = newPosX;
            part->pos.y = newPosY;
            time = 0;
        }
    }
}

Particle particles[240];
int numParticles = 240;

float lineProximityDistance = 100.0f;
float mouseProximityDistance = 200.0f;

int recommendedWidth = 1600;
int recommendedHeight = 900;

bool drawColors = true;
bool drawFPS = true;


void game_init(void)
{
    CP_System_SetWindowSize(recommendedWidth, recommendedHeight);
    CP_System_SetFrameRate(60.0f);
    //setFrameRate(60.0f);

    for (int i = 0; i < numParticles; ++i) {
        ParticleCreate(&particles[i]);
    }
}

void game_update(void)
{
    CP_Settings_BlendMode(CP_BLEND_ALPHA);
    CP_Graphics_ClearBackground(CP_Color_Create(51, 51, 51, 255));

    //CP_Graphics_ClearBackground(color((int)(mouseX / canvasWidth * 255.0f), 0, 0, 255));
    //CP_Graphics_Background(CP_CreateColor((int)(CP_Input_GetMouseX() / CP_Graphics_GetCanvasWidth() * 255.0f), 0, 0, 255));

    CP_Settings_NoStroke();
    CP_Settings_Fill(CP_Color_Create(0, 0, 0, 255));

    for (int i = 0; i < numParticles; ++i)
    {
        ParticleUpdate(&particles[i]);
        ParticleDisplay(&particles[i]);
    }

    CP_Settings_BlendMode(CP_BLEND_ADD);
    CP_Settings_StrokeWeight(3);
    CP_Color lineColor;
    float mouseX = CP_Input_GetMouseX();
    float mouseY = CP_Input_GetMouseY();

    for (int i = 0; i < numParticles; ++i)
    {
        // draw white lines from the particles to the mouse position
        float distXMouse = (float)fabsf(particles[i].pos.x - mouseX);
        float distYMouse = (float)fabsf(particles[i].pos.y - mouseY);
        if (distXMouse < mouseProximityDistance && distYMouse < mouseProximityDistance)
        {
            lineColor.r = 128;
            lineColor.g = 128;
            lineColor.b = 128;
            lineColor.a = (unsigned char)(255.0f * min(1.0f, (mouseProximityDistance - max(distXMouse, distYMouse)) / (mouseProximityDistance * 0.3f)));
            CP_Settings_Stroke(lineColor);
            CP_Graphics_DrawLine(particles[i].pos.x, particles[i].pos.y, mouseX, mouseY);
        }

        // draw lines between particles based on the additive color of both particles
        if (!drawColors)
        {
            continue;
        }
        for (int j = i + 1; j < numParticles; ++j)
        {
            float distX = (float)fabsf(particles[i].pos.x - particles[j].pos.x);
            float distY = (float)fabsf(particles[i].pos.y - particles[j].pos.y);
            if (distX < lineProximityDistance && distY < lineProximityDistance)
            {
                lineColor.r = particles[i].color->r + particles[j].color->r;
                lineColor.g = particles[i].color->g + particles[j].color->g;
                lineColor.b = particles[i].color->b + particles[j].color->b;
                lineColor.a = (unsigned char)(255.0f * min(1.0f, (lineProximityDistance - max(distX, distY)) / (lineProximityDistance * 0.3f)));
                CP_Settings_Stroke(lineColor);
                CP_Graphics_DrawLine(particles[i].pos.x, particles[i].pos.y, particles[j].pos.x, particles[j].pos.y);
            }
        }
    }

    if (CP_Input_KeyTriggered(KEY_SPACE))
    {
        drawColors = !drawColors;
    }

    // Profiling info and frameRate testing
    if (drawFPS)
    {
        CP_Settings_TextSize(20);
        CP_Settings_BlendMode(CP_BLEND_ALPHA);
        CP_Settings_Fill(CP_Color_Create(0, 0, 0, 128));
        CP_Settings_NoStroke();
        CP_Graphics_DrawRect(0, 0, 150, 30);
        CP_Settings_Fill(CP_Color_Create(255, 255, 255, 255));
        char buffer[100];
        sprintf_s(buffer, 100, "FPS: %f", CP_System_GetFrameRate());
        CP_Font_DrawText(buffer, 20, 20);
    }
}


void game_exit(void)
{

}

int main(void)
{
    CP_Engine_SetNextGameState(game_init, game_update, game_exit);
    CP_Engine_Run();
    return 0;
}
