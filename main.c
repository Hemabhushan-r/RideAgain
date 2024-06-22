#define PHYSAC_IMPLEMENTATION

#include<stdio.h>
#include<stdlib.h>
#include "raylib.h"
#include "physac.h"
#include <string.h>
#include "structs.h"

int screen_number;
int level=0;
int shop_open=0;
int pageno=1;
int score=0;
float val=0;
float portal_rotation=0.0f;
float sat=0;
int blinker=0;
int spawn_counter=0;
int sped_counter;
int processingspeedfactor=1;
int cartweight=0;
int lives=3;
char scoretext[50];
int multiplier=1;
int scoredisp_counter=0;
int score_pbar=0;
int sped_up =0;
int cartwidth =100;
int cartheight =70;
int gears_counter=0;
int score_gained_ypos;
int cart_capacity=15;
int processing=0;
float block_terminal_velocity=0.35;
int spawn_max_count=200;
// FILE * inst=NULL;
rawm* spawnedblock=NULL;
// PhysicsBody leftwall;
// PhysicsBody rightwall;
PhysicsBody cart;
Texture2D tin,graphite,brick,titanium,cement,star,barrel,steelrod,bomb,screw,speed;
Texture2D background,cart_texture,chitti_texture,portal,rupee,shop,factory_icon;
Texture2D instructions_texture,cart_filled,page1,page2,ground_texture;
Music intro,bgm,burst,cha_ching;
Texture2D gears_gif[12];
// PhysicsBody player;
// PhysicsBody ground;
int screenWidth=800,screenHeight=600;
int r2=-1;
int process_counter=0;
int jam_chance;
rawm* spawn(int r2);
void InitializeGame();
void block_recieved();
void blink(int r2);
void scoredisp();
void LoadTexturesFromFiles();
void DrawGIF(Texture2D [],int*,int,int,float,int,int);
int main(){

InitWindow(screenWidth,screenHeight,"Cyclonus");
InitPhysics();
InitAudioDevice();
intro = LoadMusicStream("assets/audio/intro.mp3");

screen_number=1;


SetTargetFPS(60);

while (!WindowShouldClose())
{

    // Updation
Vector2 mv=GetMousePosition();
if(screen_number==2) 
if(IsMouseButtonPressed(MOUSE_BUTTON_LEFT)&&CheckCollisionPointRec(mv,(Rectangle){550,40,250,110})){
    shop_open^=1;
    puts("shop");
}
if(IsKeyPressed(KEY_F11))
ToggleFullscreen();

if(screen_number==4){

if(IsKeyPressed(KEY_LEFT))
pageno--;

if(pageno<1) pageno=1;
if(IsKeyPressed(KEY_RIGHT))
pageno++;
if(pageno>2) pageno=2;
if(IsKeyPressed(KEY_BACKSPACE))
screen_number=1;

}

if(screen_number==1){
    if(CheckCollisionPointRec(mv,(Rectangle){35,60,130,85}) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
    screen_number=4;
}

// printf("\n%d",scoredisp_counter );
if(screen_number==2 && shop_open){
  if(spawnedblock!=NULL)  {
DestroyPhysicsBody(spawnedblock->body);
free(spawnedblock);
spawnedblock=NULL;
}
spawn_counter=0;

if(IsKeyPressed(KEY_BACKSPACE))
shop_open^=1;

if(CheckCollisionPointRec(mv, (Rectangle){30,140,170,35}) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && score-70>=0 &&  processingspeedfactor<5){

    
processingspeedfactor++;
score=score-70;
printf("processing speed increased to : %dx",processingspeedfactor);
}
if(CheckCollisionPointRec(mv, (Rectangle){280,140,100,35}) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && score-150>=0){

lives++;
score-=150;
printf("life added ");
}
if(CheckCollisionPointRec(mv, (Rectangle){480,140,585,200}) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && score-50>=0){

score-=50;
printf("cart-capacity upgrade by 5 ");
cart_capacity+=5;
}

}
if(screen_number==2  && !shop_open) {

// music
// if(spawnedblock!=NULL)
// printf("\ncurrent torque : %f",spawnedblock->body->torque);
if(IsMusicStreamPlaying(bgm))
    UpdateMusicStream(bgm);
else PlayMusicStream(bgm);

if(IsMusicStreamPlaying(burst)){
if(GetMusicTimePlayed(burst)>=GetMusicTimeLength(burst) -0.1)
StopMusicStream(burst);
else
    UpdateMusicStream(burst);
}

if(!shop_open)
UpdatePhysics();

//cheatcode

if(IsKeyPressed(KEY_C))
score+=100;

if(processing==-1 && GetMouseWheelMove())
processing=1;


// printf("bodies: %d\n",GetPhysicsBodiesCount() );

if(cart_capacity==15){
level=1;
spawn_max_count=200;
}

if(cart_capacity>=35){
level=2;
spawn_max_count=145;
}
if(cart_capacity>=50){
level=3;
spawn_max_count=85;
block_terminal_velocity=0.5;
// SetPhysicsGravity(0,15);
}


if(lives<=0)
screen_number=3;

if(mv.x>600 && IsMouseButtonDown(MOUSE_RIGHT_BUTTON) && processing==0 && cartweight>0) {
    processing=1; 
    process_counter=cartweight*20; 
    score+=cartweight*multiplier; 
    scoredisp_counter=200;
    score_pbar=process_counter;
    strcpy(scoretext , TextFormat("+%d x %d",cartweight,multiplier));
    
    cartweight=0; 
    PlayMusicStream(cha_ching);

    }

if(processing==1){

    if(IsMusicStreamPlaying(cha_ching)){
        UpdateMusicStream(cha_ching);
    if(GetMusicTimePlayed(cha_ching)>=GetMusicTimeLength(cha_ching)-0.1)
    StopMusicStream(cha_ching);
    }
   

    jam_chance=GetRandomValue(1,805);

    if(jam_chance==10){
        if(sped_up==0)
    processing=-1;
    jam_chance=1;
    }

    process_counter-=1*processingspeedfactor;
    if(process_counter<0){
    process_counter=0;
    score_pbar=0;
    }

    if(process_counter==0)
    processing=0;
}


if(spawn_counter<spawn_max_count){
if(!shop_open)
spawn_counter++;
}
else{
spawn_counter=0;
r2=GetRandomValue(1,3);
blink(r2);
if(spawnedblock!=NULL){
DestroyPhysicsBody(spawnedblock->body);
free(spawnedblock);
}
spawnedblock= spawn(r2);
}
// if(IsKeyPressed(KEY_SPACE)){

// player->velocity.y=-2;

//     puts("jump");
// }

cart->position=mv;

// if(IsKeyDown(KEY_D)){

// player->velocity.x=0.8;

// }
// if(IsKeyDown(KEY_A)){

// player->velocity.x=-0.8;

// }

if(spawnedblock!=NULL){

if(spawnedblock->body->velocity.y>block_terminal_velocity)
spawnedblock->body->velocity.y=block_terminal_velocity;

if(CheckCollisionRecs ((Rectangle){spawnedblock->body->position.x-25,spawnedblock->body->position.y-25,50,50},(Rectangle){cart->position.x-cartwidth/2 ,cart->position.y-cartheight/2,cartwidth,cartheight})){
    block_recieved();
}
// checks for collison with ground
else if (CheckCollisionRecs ((Rectangle){spawnedblock->body->position.x-25,spawnedblock->body->position.y-25,50,50},(Rectangle){0,screenHeight-20,screenWidth,20})){
    puts("fell to ground");
    if(! (spawnedblock->index==9 || spawnedblock->index==10 ))
    lives--;
    DestroyPhysicsBody( spawnedblock->body);
    free(spawnedblock);
    spawnedblock=NULL;

}
}
if(cart->position.x>550-50)
{cart->position.x=550-50 ;
cart->velocity.x=0;}
if(cart->position.x<=55)
{cart->position.x=55;
cart->velocity.x=0;}

}

// printf("%d\n",GetPhysicsBodiesCount() );

    BeginDrawing();

    ClearBackground(BLACK);
//the welcome screen
    if(screen_number==1){
    ClearBackground(ColorFromHSV(190,sat,val));
    val+=0.005;
    sat+=0.001;
    if(val>20)
    val=2;
    if(sat>2)
    sat=0;

    if(IsMusicStreamPlaying(intro))
    UpdateMusicStream(intro);
 
    else PlayMusicStream(intro);

    DrawText("Cyclonus",100,150,50,BLACK);
   
    DrawText("[ Best played in fullscreen mode ] \n hit F11 to go to fullscreen mode ",100,220,24,BLACK);
    
    if(blinker<30)
    DrawText("Click here for HOW-TO-PLAY",20,20,26,BLACK);

    if(instructions_texture.width==0)
    instructions_texture=LoadTexture("assets/instructions.png");
    DrawTexture(instructions_texture,35,60,WHITE);


    if(chitti_texture.height==0)
    chitti_texture = LoadTexture("assets/chitti.png");
    DrawTextureEx(chitti_texture,(Vector2){450,30},0,0.8f,WHITE);

    DrawText("Powered by raylib",100,500,30,BLACK);

    
    if(blinker<30)
    DrawText("Click ENTER TO START!",450,560,26,BLACK);
  blinker++;
  if(blinker>=60) blinker=0;
    
if(IsKeyPressed(KEY_ENTER)){

    InitializeGame();
}


    }
  //the game screen  
    if(screen_number==2 && !shop_open){

ClearBackground(DARKGRAY);
DrawTexture(background,0,0,DARKGRAY);
DrawTexture(ground_texture,0,screenHeight-28,WHITE);

// Draw created physics bodies
// int bodiesCount = GetPhysicsBodiesCount();
// for (int i = 0; i < bodiesCount; i++)
// {
//     PhysicsBody body = GetPhysicsBody(i);

//     int vertexCount = GetPhysicsShapeVerticesCount(i);
//     for (int j = 0; j < vertexCount; j++)
//     {
//         // Get physics bodies shape vertices to draw lines
//         // Note: GetPhysicsShapeVertex() already calculates rotation transformations
//         Vector2 vertexA = GetPhysicsShapeVertex(body, j);

//         int jj = (((j + 1) < vertexCount) ? (j + 1) : 0);   // Get next vertex or first to close the shape
//         Vector2 vertexB = GetPhysicsShapeVertex(body, jj);

//         DrawLineV(vertexA, vertexB, WHITE);     // Draw a line between two vertex positions
//     }
// }

DrawTextureEx(shop,(Vector2){550,40},0,250.0/shop.width,WHITE);

if(sped_up){
DrawTextureEx(speed,(Vector2){20,170},0,80.0/speed.width,WHITE);
DrawText(TextFormat ("%d",sped_counter/60),30,228,38,WHITE);
sped_counter--;
if(sped_counter<=0){
    sped_counter=0;
    sped_up=0;
    block_terminal_velocity-=0.4;
}
}

//draw cart and chitti
if(cartweight==0)
DrawTextureEx(cart_texture,MathVector2Subtract( cart->position,(Vector2){75,75}),cart->orient * RAD2DEG,150.0/cart_texture.width,WHITE);
else
DrawTextureEx(cart_filled,MathVector2Subtract( cart->position,(Vector2){75,75}),cart->orient * RAD2DEG,150.0/cart_filled.width,WHITE);

// DrawTextureEx(chitti_texture,MathVector2Subtract( player->position,(Vector2){35,60}),player->orient,150.0/cart_texture.width,WHITE);


//drawtextures for blocks
if(spawnedblock!=NULL){
    if(spawnedblock->index==1)
DrawTextureEx(tin,MathVector2Subtract( spawnedblock->body->position,(Vector2){30,30}),spawnedblock->body->orient*RAD2DEG,60.0/(float)tin.width,WHITE);
    if(spawnedblock->index==2)
DrawTextureEx(graphite,MathVector2Subtract( spawnedblock->body->position,(Vector2){30,30}),spawnedblock->body->orient*RAD2DEG,60.0/(float)graphite.width,WHITE);
    if(spawnedblock->index==3)
DrawTextureEx(titanium,MathVector2Subtract( spawnedblock->body->position,(Vector2){30,30}),spawnedblock->body->orient*RAD2DEG,60.0/(float)titanium.width,WHITE);
    if(spawnedblock->index==4)
DrawTextureEx(screw,MathVector2Subtract( spawnedblock->body->position,(Vector2){30,30}),spawnedblock->body->orient*RAD2DEG + 60,38.0/(float)screw.width,WHITE);
    if(spawnedblock->index==5)
DrawTextureEx(steelrod,MathVector2Subtract( spawnedblock->body->position,(Vector2){30,30}),spawnedblock->body->orient*RAD2DEG,60.0/(float)steelrod.width,WHITE);
    if(spawnedblock->index==6)
DrawTextureEx(brick,MathVector2Subtract( spawnedblock->body->position,(Vector2){30,30}),spawnedblock->body->orient*RAD2DEG,60.0/(float)brick.width,WHITE);
    if(spawnedblock->index==7)
DrawTextureEx(barrel,MathVector2Subtract( spawnedblock->body->position,(Vector2){30,30}),spawnedblock->body->orient*RAD2DEG,60.0/(float)barrel.width,WHITE);
    if(spawnedblock->index==8)
DrawTextureEx(cement,MathVector2Subtract( spawnedblock->body->position,(Vector2){30,30}),spawnedblock->body->orient*RAD2DEG,60.0/(float)cement.width,WHITE);
    if(spawnedblock->index==9)
DrawTextureEx(star,MathVector2Subtract( spawnedblock->body->position,(Vector2){30,30}),spawnedblock->body->orient*RAD2DEG,60.0/(float)star.width,WHITE);
    if(spawnedblock->index==10)
DrawTextureEx(bomb,MathVector2Subtract( spawnedblock->body->position,(Vector2){40,40}),spawnedblock->body->orient*RAD2DEG,80.0/(float)bomb.width,WHITE);
    if(spawnedblock->index==11)
DrawTextureEx(speed,MathVector2Subtract( spawnedblock->body->position,(Vector2){30,30}),spawnedblock->body->orient*RAD2DEG,90.0/(float)speed.width,WHITE);
}

if(cartweight>=7.0f*cart_capacity/10.0f)
DrawText("You might want to dump the cart before \ncollecting more materials",150,300,20,GRAY);
DrawText(TextFormat("Cart weight: %d/%d",cartweight,cart_capacity),screenWidth-240,screenHeight - 58,24,WHITE);
DrawText(TextFormat("lives left : %d",lives),screenWidth-400,18,25,WHITE);
DrawTextureEx(rupee,(Vector2){28,6},0,38.0 / rupee.width ,WHITE);
DrawText(TextFormat(" %d %dx",score,multiplier),66,18,25,WHITE);
DrawText(TextFormat("current level : %d",level),screenWidth-640,18,25,WHITE);


if(processing==0){
DrawRectangle(590,370,150,150,GOLD);
scoredisp();
DrawGIF(gears_gif,&gears_counter,590,370,150,12,0);
}
else if(processing==1)
{DrawText("processing...",600,350,20,DARKGREEN);
 scoredisp();
DrawRectangle(590,370,150,150,DARKGREEN);
DrawGIF(gears_gif,&gears_counter,590,370,150,12,1);
// DrawText(TextFormat("%d",process_counter),635,470,25,BLACK);
if(score_pbar>0)
DrawRectangle(600,490,125 * process_counter/score_pbar,15,YELLOW);
}
else if(processing==-1){
DrawText("Processor has been jammed.\ncrank the gears by wheel to repair",570,285,16,WHITE);
DrawRectangle(590,370,150,150,RED);
scoredisp();
DrawGIF(gears_gif,&gears_counter,590,370,150,12,0);
// DrawText(TextFormat("%d",process_counter),635,470,25,BLACK);
}


// DrawCircle(40,ground->position.y-8,8,BLUE);
// DrawCircle(550,ground->position.y-8,8,BLUE);

DrawTexturePro(portal,(Rectangle){0,0,portal.width,portal.height},(Rectangle){100,100,100,100},(Vector2){50,50},portal_rotation,WHITE);
DrawTexturePro(portal,(Rectangle){0,0,portal.width,portal.height},(Rectangle){300,100,100,100},(Vector2){50,50},portal_rotation,WHITE);
DrawTexturePro(portal,(Rectangle){0,0,portal.width,portal.height},(Rectangle){500,100,100,100},(Vector2){50,50},portal_rotation,WHITE);

if(sped_up==0)
portal_rotation++;
else portal_rotation+=2;
if(portal_rotation>=360.0)
portal_rotation=0;

if(spawnedblock!=NULL){



}




    }
    // shop 
if(screen_number==2 && shop_open){
ClearBackground(RED);

DrawText("SHOP",screenWidth/2-200,30,60,YELLOW);
DrawTextureEx(factory_icon,(Vector2){16,screenHeight- 130},0,155.0/factory_icon.width,WHITE);
DrawText("Click BACKSPACE to go back",180,screenHeight-30,24,BLACK);
DrawTextureEx(rupee,(Vector2){screenWidth- 166,46},0,44.0 / rupee.width ,WHITE);
DrawText(TextFormat(" %d",score),screenWidth-126,50,30,YELLOW);

if(processingspeedfactor<5){
DrawText("upgrade processor",30,140,25,YELLOW);
DrawText("70 pts",30,170,20,YELLOW);
DrawText(TextFormat("current speed : %dx",processingspeedfactor),30,200,20,YELLOW);

}
else{
DrawText("maxed out \nprocessor level",30,140,25,DARKGRAY);
// DrawText("70 pts",30,170,20,DARKGRAY);
DrawText(TextFormat("current speed : %dx",processingspeedfactor),30,200,20,DARKGRAY);
}

DrawText("add a life",280,140,25,YELLOW);    
DrawText("150 pts",280,170,20,YELLOW);
DrawText(TextFormat("current lives : %d",lives),280,200,20,YELLOW);

DrawText("increase cart-capacity by 5",480,140,22,YELLOW);    
DrawText("50 pts",480,170,20,YELLOW);
DrawText(TextFormat("current capacity : %d",cart_capacity),480,200,20,YELLOW);
if(cart_capacity==30 || cart_capacity==45)
DrawText(TextFormat("NOTE : You will be promoted to\n level %d worker after this upgrade",level+1),480,230,18,BLACK);




}

// instructions
if(screen_number==4){

if(IsMusicStreamPlaying(intro))
    UpdateMusicStream(intro);
 
    else PlayMusicStream(intro);

ClearBackground(BROWN);
if(page1.width==0)
page1 = LoadTexture("assets/page1.png");
if(page2.width==0)
page2 = LoadTexture("assets/page2.png");
if(pageno==1)
DrawTexturePro(page1,(Rectangle){0,0,page1.width,page1.height},(Rectangle){24,30,screenWidth-40,screenHeight-20},(Vector2){0,0},0,WHITE);
else
DrawTexturePro(page2,(Rectangle){0,0,page1.width,page1.height},(Rectangle){24,30,screenWidth-40,screenHeight-20},(Vector2){0,0},0,WHITE);

DrawText("use arrow keys to navigate pages, BACKSPACE for home screen",10,5,24,WHITE);


}


    //endgame
    if(screen_number==3){
        ClearBackground(DARKGRAY);
        DrawText(TextFormat("You reached level :%d \n and scored %d ",level,score),150,300,50,RED);
        DrawText("Click enter to play again",150,510,40,WHITE);


        if(IsKeyPressed(KEY_ENTER)){
            screen_number=2;
            ClosePhysics();
            InitPhysics();
        InitializeGame();
        }
    }
    
    
    EndDrawing();

}





CloseAudioDevice();
ClosePhysics();
CloseWindow();



return 0;
}

void InitializeGame(){

LoadTexturesFromFiles();
spawn_counter=0;
cartweight=0;
lives=3;
sped_up=0;
processingspeedfactor=1;
level=1;
score=0;
shop_open=0;
multiplier=1;
cart_capacity=15;
process_counter=0;
block_terminal_velocity=0.35;
processing=0;
spawnedblock=NULL;
screen_number=2;
// SetPhysicsGravity(0,9.8);

// player= CreatePhysicsBodyRectangle((Vector2){25,200},50,150,30);
// player->enabled=true;
// player->useGravity=true;
// player->restitution=0.6;
// player->freezeOrient=true;

// ground=CreatePhysicsBodyRectangle((Vector2){screenWidth/2,screenHeight-30},screenWidth,20,10);
// ground->enabled=false;
// ground->useGravity=false;
// ground->dynamicFriction=0.3;
// ground->staticFriction=0.3;
cart = CreatePhysicsBodyRectangle((Vector2){100,screenHeight/2},cartwidth,cartheight,65);
cart->enabled=true;
cart->useGravity=true;
cart->freezeOrient=true;
cart->restitution=0.7;
// cart->dynamicFriction=0.2;
// cart->staticFriction=0.2;
// cart->dynamicFriction=0.5;
// cart->staticFriction=0.5;
// leftwall=CreatePhysicsBodyRectangle((Vector2){0,screenHeight/2},20,screenHeight,10);
// leftwall->enabled=false;
// leftwall->useGravity=false;
// rightwall=CreatePhysicsBodyRectangle((Vector2){screenWidth,screenHeight/2},20,screenHeight,10);
// rightwall->enabled=false;
// rightwall->useGravity=false;

}

void blink(int r2){

if(r2==1){
    DrawCircle(100,100,6,RED);
}

if(r2==2){
    DrawCircle(300,100,6,RED);
}

if(r2==3){
    DrawCircle(500,100,6,RED);
}

}



rawm* spawn(int r2){
int r;
int sb_chance=-1;

if (level==1)
r = GetRandomValue(1,4);
else if(level==2)
r= GetRandomValue(1,6);
else if(level==3)
r= GetRandomValue(1,8);
else r=-1;

sb_chance=GetRandomValue(1,60);
if(sb_chance==5 && multiplier<3)
r=9;
if(sb_chance==6 || sb_chance==7 || sb_chance==8 ||sb_chance==9 || sb_chance==10 || sb_chance==11)
r=10;
if(sb_chance==4 && sped_up==0)
r=11;

if(IsKeyDown(KEY_B))
r=11;

printf("Spawned : %d\n", r);
rawm* blk = (rawm*) malloc(sizeof(rawm));
blk->body=CreatePhysicsBodyRectangle((Vector2){0,0},40,40,10);
blk->index=r;


if(r==1){


blk->weight=5;

}
if(r==2){


blk->weight=10;

}
if(r==3){


blk->weight=7;

}
if(r==4) blk->weight=6;
if(r==5) blk->weight=16;
if(r==6) blk->weight=12;
if(r==7) blk->weight=22;
if(r==8) blk->weight=18;
if(r==9) blk->weight=0;
if(r==10) blk->weight=0;
if(r==11) blk->weight=0;

if(r2==1)
blk->body->position=(Vector2){100,100};
if(r2==2)
blk->body->position=(Vector2){300,100};
if(r2==3)
blk->body->position=(Vector2){500,100};



blk->body->enabled=true;
blk->body->useGravity=true;
blk->body->mass=1;
PhysicsAddTorque(blk->body,GetRandomValue(-1500.0f,1500.0f));
// printf("\nadded torque : %f",blk->body->torque);
r2=-1;
return blk;
}

void block_recieved(){

if(cartweight+spawnedblock->weight>cart_capacity)
return;

cartweight+=spawnedblock->weight;

if(spawnedblock->index==9){
puts("multiplier gained");
multiplier++;if(multiplier>=3) multiplier=3;
}

if(spawnedblock->index==10){
    lives--;
// score-=40;
if(score<0)
score=0;
PlayMusicStream(burst);
multiplier=1;
cart->velocity.y=-2;
cart->velocity.x=0.3*GetRandomValue(-5,5);
puts("kaboom!");
}

if(spawnedblock->index==11){
    block_terminal_velocity+=0.4;
    sped_up=1;
    sped_counter = 30*60;
    puts("sped up!!");
}


DestroyPhysicsBody(spawnedblock->body);

free(spawnedblock);
spawnedblock=NULL;
puts("freed!");



}

void LoadTexturesFromFiles(){

tin = LoadTexture("assets/materials/tin-ingot.png");
screw = LoadTexture("assets/materials/screw.png");
graphite = LoadTexture("assets/materials/graphite.png");
titanium = LoadTexture("assets/materials/titanium.png");
star = LoadTexture("assets/materials/star.png");
barrel = LoadTexture("assets/materials/barrel.png");
cement = LoadTexture("assets/materials/cement.png");
brick = LoadTexture("assets/materials/brick.png");
steelrod = LoadTexture("assets/materials/steelrod.png");
shop = LoadTexture("assets/shop_icon.png");
factory_icon = LoadTexture("assets/factory_icon.png");
bomb = LoadTexture("assets/materials/bomb.png");
background = LoadTexture("assets/background.png");
speed = LoadTexture("assets/materials/speed.png");
cart_texture = LoadTexture("assets/cart.png");
chitti_texture = LoadTexture("assets/chitti.png");
portal = LoadTexture("assets/portal.png");
ground_texture = LoadTexture("assets/factory-floor.png");
cart_filled = LoadTexture("assets/cart-filled.png");
rupee = LoadTexture("assets/rupee.png");


bgm= LoadMusicStream("assets/audio/bgm.mp3");
burst= LoadMusicStream("assets/audio/burst.wav");
cha_ching= LoadMusicStream("assets/audio/cha_ching.mp3");

for(int i=0;i<12;i++)
gears_gif[i]=LoadTexture(TextFormat("assets/gifs/gears/%d.png",i+1));

}

void DrawGIF(Texture2D gif[],int* pos,int x,int y,float w,int max,int running){

DrawTextureEx(gif[*pos],(Vector2){x,y},0, w/gif[0].width,WHITE);
// printf("\ndrew gif : %d", *pos);

if(running){
    (*pos)++;
    if(*pos>=max)
    *pos=0;
}
}
void scoredisp(){

if(scoredisp_counter==0)
return;

DrawText(scoretext,610,200+scoredisp_counter/4,40,ColorAlpha(YELLOW,scoredisp_counter/200.0f));
scoredisp_counter--;
}