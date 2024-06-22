#define PHYSAC_IMPLEMENTATION

#include <stdio.h>
#include <stdlib.h>
#include "raylib.h"
#include "physac.h"
#include <process.h>

Texture2D cyclonus;
Texture2D rideagain;
Texture2D tetris;

int main(){



InitWindow(800,600,"3-in-one games");
SetTargetFPS(60);

 cyclonus = LoadTexture("assets/chitti.png");
 rideagain = LoadTexture("assets/rideagain.png");
 tetris = LoadTexture("assets/tetris.png");
Rectangle r1 = (Rectangle){50,100,200,250};
Rectangle r2 = (Rectangle){300,100,200,150};
Rectangle r3 = (Rectangle){550,100,200,150};
Vector2 mv;
while (!WindowShouldClose())
{
    mv=GetMousePosition();

if(IsMouseButtonPressed(MOUSE_BUTTON_LEFT)&&CheckCollisionPointRec(mv,r1)) 
execl("Cyclonus.exe","Cyclonus.exe",NULL) ;

if(IsMouseButtonPressed(MOUSE_BUTTON_LEFT)&&CheckCollisionPointRec(mv,r2)) 
execl("RideAgain/game.exe","RideAgain/game.exe",NULL) ;

if(IsMouseButtonPressed(MOUSE_BUTTON_LEFT)&&CheckCollisionPointRec(mv,r3)) 
execl("tetris.exe","tetris.exe",NULL) ;




BeginDrawing();
ClearBackground(BLACK);


DrawTexturePro(cyclonus,(Rectangle){0,0,cyclonus.width,cyclonus.height},r1,(Vector2){0,0},0,WHITE);
DrawText("Cyclonus",65,375,30,WHITE);
DrawTexturePro(rideagain,(Rectangle){0,0,rideagain.width,rideagain.height},r2,(Vector2){0,0},0,WHITE);
DrawText("rideagain",365,375,30,WHITE);
DrawTexturePro(tetris,(Rectangle){0,0,tetris.width,tetris.height},r3,(Vector2){0,0},0,WHITE);
DrawText("tetris",565,375,30,WHITE);


DrawText("Pick a game",255,510,40,WHITE);


EndDrawing();
}

CloseWindow();

return 0;
}