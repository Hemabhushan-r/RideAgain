#define PHYSAC_IMPLEMENTATION

#include<stdio.h>
#include<stdlib.h>
#include "raylib.h"
#include "physac.h"




int main(){

InitWindow(800,600,"'PES'ky Chitti-tril");
InitPhysics();
InitAudioDevice();

PhysicsBody rect=CreatePhysicsBodyRectangle((Vector2){300,300},150,150,10);
PhysicsBody ground =CreatePhysicsBodyRectangle((Vector2){GetScreenWidth()/2,GetScreenHeight()},GetScreenWidth(), 6,10);
PhysicsBody lwall =CreatePhysicsBodyRectangle((Vector2){5,GetScreenHeight()/2},10, GetScreenHeight(),10);
PhysicsBody rwall =CreatePhysicsBodyRectangle((Vector2){GetScreenWidth()-5,GetScreenHeight()/2},10, GetScreenHeight(),10);

PhysicsBody ball = CreatePhysicsBodyCircle((Vector2){500,500},20,10);

ground->enabled=false;
lwall->enabled=false;
rwall->enabled=false;

lwall->restitution=3;
ball->restitution=3;
rwall->restitution=3;

rect->useGravity=true;

SetTargetFPS(60);
SetPhysicsGravity(0,9.8);
// SetPhysicsGravity(0,0.5);
while (!WindowShouldClose())
{
    //Updation
    UpdatePhysics();




    if(IsKeyDown(KEY_RIGHT))
        rect->velocity.x=1;
    if(IsKeyDown(KEY_LEFT))
     rect->velocity.x=-1;
    if(IsKeyDown(KEY_SPACE))
    rect->velocity.y=-2;



    BeginDrawing();
    ClearBackground(WHITE);

   
   DrawBody(rect);
   DrawBody(ground);
   DrawBody(lwall);
   DrawBody(rwall);
   DrawBody(ball);

    EndDrawing();

}





CloseAudioDevice();
ClosePhysics();
CloseWindow();



return 0;
}

void DrawBody(PhysicsBody body){

     PhysicsVertexData vd= body->shape.vertexData;
    for (int i =0;i<vd.vertexCount-1;i++)
        DrawLineV(MathVector2Add(vd.positions[i],body->position),MathVector2Add(vd.positions[i+1],body->position),BLACK);
    DrawLineV(MathVector2Add(vd.positions[vd.vertexCount -1],body->position),MathVector2Add(vd.positions[0],body->position),BLACK);
}