#include "raylib.h"
#include "raymath.h"
#include "raylibODE.h"
#include <string.h>
#include "ode/ode.h"
#include "assert.h"
#include "rlgl.h"

#define RLIGHTS_IMPLEMENTATION
#include "rlights.h"



#define MAX_COLUMNS 20


static geomInfo disabled;

#if defined(PLATFORM_DESKTOP)
  #define GLSL_VERSION 330
#else
  #define GLSL_VERSION 100
#endif

bool checkColliding(dGeomID g)
{
    geomInfo* gi = (geomInfo*)dGeomGetData(g);
    if (!gi) return true;
    return gi->collidable;
}

void MyDrawModel(Model model, Color tint)
{

    for (int i = 0; i < model.meshCount; i++)
    {
        Color color = model.materials[model.meshMaterial[i]].maps[MATERIAL_MAP_DIFFUSE].color;

        Color colorTint = WHITE;
        colorTint.r = (unsigned char)((((float)color.r / 255.0) * ((float)tint.r / 255.0)) * 255.0f);
        colorTint.g = (unsigned char)((((float)color.g / 255.0) * ((float)tint.g / 255.0)) * 255.0f);
        colorTint.b = (unsigned char)((((float)color.b / 255.0) * ((float)tint.b / 255.0)) * 255.0f);
        colorTint.a = (unsigned char)((((float)color.a / 255.0) * ((float)tint.a / 255.0)) * 255.0f);

        model.materials[model.meshMaterial[i]].maps[MATERIAL_MAP_DIFFUSE].color = colorTint;
        DrawMesh(model.meshes[i], model.materials[model.meshMaterial[i]], model.transform);
        model.materials[model.meshMaterial[i]].maps[MATERIAL_MAP_DIFFUSE].color = color;
    }
}

void rayToOdeMat(Matrix* m, dReal* R) {
    R[0] = m->m0;   R[1] = m->m4;   R[2] = m->m8;    R[3] = 0;
    R[4] = m->m1;   R[5] = m->m5;   R[6] = m->m9;    R[7] = 0;
    R[8] = m->m2;   R[9] = m->m6;   R[10] = m->m10;   R[11] = 0;
    R[12] = 0;       R[13] = 0;       R[14] = 0;        R[15] = 1;
}

void odeToRayMat(const dReal* R, Matrix* m)
{
    m->m0 = R[0];  m->m1 = R[4];  m->m2 = R[8];      m->m3 = 0;
    m->m4 = R[1];  m->m5 = R[5];  m->m6 = R[9];      m->m7 = 0;
    m->m8 = R[2];  m->m9 = R[6];  m->m10 = R[10];    m->m11 = 0;
    m->m12 = 0;    m->m13 = 0;    m->m14 = 0;        m->m15 = 1;
}

void drawGeom(dGeomID geom,int CarBtrue,int CarLWtrue)
{
    const dReal* pos = dGeomGetPosition(geom);
    const dReal* rot = dGeomGetRotation(geom);
    int class = dGeomGetClass(geom);
    Model* m = 0;
    Model* mref = 0;
    dVector3 size;
    if (class == dBoxClass) {
        m = &box;
        mref = &AM;
        dGeomBoxGetLengths(geom, size);
        //DrawModelEx(AM, (Vector3) { pos[0], pos[1], pos[2] }, (Vector3) { 1.0, 0.0, 0.0 }, 90.0, (Vector3) { 1.0, 1.0, 1.0 }, WHITE);
    }
    else if (class == dSphereClass) {
        m = &ball;
        float r = dGeomSphereGetRadius(geom);
        size[0] = size[1] = size[2] = (r * 2);
    }
    else if (class == dCylinderClass) {
        if (!CarLWtrue) {
            m = &WheelAMDCR;
        }
        else {
            m = &WheelAMDCL;
        }
        dReal l, r;
        dGeomCylinderGetParams(geom, &r, &l);
        size[0] = size[1] = r * 2;
        size[2] = l;
    }
    if (!m) return;

    Matrix matScale = MatrixScale(size[0], size[1], size[2]);
    Matrix matRot;
    odeToRayMat(rot, &matRot);
    Matrix matTran = MatrixTranslate(pos[0], pos[1], pos[2]);

    m->transform = MatrixMultiply(MatrixMultiply(matScale, matRot), matTran);
    if (class == dBoxClass && CarBtrue) {
        Matrix matTran = MatrixTranslate(pos[0], pos[1]-0.3, pos[2]);
        mref->transform = MatrixMultiply(MatrixMultiply(MatrixScale(1.0,1.0,1.0), matRot), matTran);
    }
    dBodyID b = dGeomGetBody(geom);
    Color c = WHITE;
    if (b) if (!dBodyIsEnabled(b)) c = RED;
    
    //MyDrawModel(*m, c);
   
    if (class == dBoxClass && CarBtrue) {
        MyDrawModel(*mref, c);
        mref->transform = MatrixIdentity();
    }
    else {
        if (class != dBoxClass) {
            MyDrawModel(*m, c);
        }
    }
    m->transform = MatrixIdentity();
    //mref->transform = MatrixIdentity();
}
typedef enum CarModel { Amuscle,MARUSSIA,BMW} CarModel;

void drawGeomTWOP(dGeomID geom, int CarBtrue, int CarLWtrue,CarModel carMP,CarModel WheelMP)
{
    const dReal* pos = dGeomGetPosition(geom);
    const dReal* rot = dGeomGetRotation(geom);
    int class = dGeomGetClass(geom);
    Model* m = 0;
    Model* mref = 0;
    dVector3 size;
    if (class == dBoxClass) {
        m = &box;
        if (carMP == Amuscle) {
            mref = &AM;
        }
        else if (carMP == MARUSSIA) {
            mref = &Marussia;
        }
        else if (carMP == BMW) {
            mref = &BMWi8;
        }
        dGeomBoxGetLengths(geom, size);
        //DrawModelEx(AM, (Vector3) { pos[0], pos[1], pos[2] }, (Vector3) { 1.0, 0.0, 0.0 }, 90.0, (Vector3) { 1.0, 1.0, 1.0 }, WHITE);
    }
    else if (class == dSphereClass) {
        m = &ball;
        float r = dGeomSphereGetRadius(geom);
        size[0] = size[1] = size[2] = (r * 2);
    }
    else if (class == dCylinderClass) {
        if (CarLWtrue) {

            if (WheelMP == Amuscle) {
                m = &WheelAMDCL;
            }
            else if (WheelMP == MARUSSIA) {
                m = &WheelMarussiaL;
            }
            else if (WheelMP == BMW) {
                m = &WheelBMWi8L;
            }

            
        }
        else {
            if (WheelMP == Amuscle) {
                m = &WheelAMDCR;
            }
            else if (WheelMP == MARUSSIA) {
                m = &WheelMarussiaR;
            }
            else if (WheelMP == BMW) {
                m = &WheelBMWi8R;
            }
        }
        dReal l, r;
        dGeomCylinderGetParams(geom, &r, &l);
        size[0] = size[1] = r * 2;
        size[2] = l;
    }
    if (!m) return;

    Matrix matScale = MatrixScale(size[0], size[1], size[2]);
    Matrix matRot;
    odeToRayMat(rot, &matRot);
    Matrix matTran = MatrixTranslate(pos[0], pos[1], pos[2]);

    m->transform = MatrixMultiply(MatrixMultiply(matScale, matRot), matTran);
    if (class == dBoxClass && CarBtrue) {
        Matrix matTran = MatrixTranslate(pos[0], pos[1] - 0.3, pos[2]);
        mref->transform = MatrixMultiply(MatrixMultiply(MatrixScale(1.0, 1.0, 1.0), matRot), matTran);
    }
    dBodyID b = dGeomGetBody(geom);
    Color c = WHITE;
    if (b) if (!dBodyIsEnabled(b)) c = RED;

    //MyDrawModel(*m, c);

    if (class == dBoxClass && CarBtrue) {
        MyDrawModel(*mref, c);
        mref->transform = MatrixIdentity();
    }
    else {
        if (class != dBoxClass) {
            MyDrawModel(*m, c);
        }
    }
    m->transform = MatrixIdentity();
    //mref->transform = MatrixIdentity();
}

//carPlayer->geoms[0],carPlayer1->geoms[3],carPlayer1->geoms[1]  dGeomID CarBodyID,dGeomID CarWFL,dGeomID CarWRL
void drawAllSpaceGeoms(dSpaceID space,vehicle *car)
{
    int ng = dSpaceGetNumGeoms(space);
    int CarBTrue=0;
    int CarLWTrue = 0;
    for (int i = 0; i < ng; i++) {
        dGeomID geom = dSpaceGetGeom(space, i);
        if (geom == car->geoms[0]) {
            CarBTrue = 1;
        }
        else if (geom == car->geoms[3] || geom == car->geoms[1]) {
            CarLWTrue = 1;
        }
        else {
            CarBTrue = 0;
            CarLWTrue = 0;
        }
        if (checkColliding(geom))
        {
            // hide non colliding geoms (car counter weights)
            drawGeom(geom,CarBTrue,CarLWTrue);
        }
    }
}

void drawAllSpaceTWOPGeoms(dSpaceID space, vehicle* carPlayer1,vehicle* carPlayer2,CarModel carMP1,CarModel carMP2)
{
    int ng = dSpaceGetNumGeoms(space);
    int CarBTrue = 0;
    int CarLWTrue = 0;
    CarModel current=Amuscle;
    CarModel currentWheel=Amuscle;
    for (int i = 0; i < ng; i++) {
        dGeomID geom = dSpaceGetGeom(space, i);
        if (geom == carPlayer1->geoms[0]) {
            CarBTrue = 1;
            current = carMP1;
            currentWheel = carMP1;
        }
        else if (geom == carPlayer1->geoms[3] || geom == carPlayer1->geoms[1]) {
            CarLWTrue = 1;
            currentWheel = carMP1;
        }
        else if (geom == carPlayer2->geoms[0]) {
            CarBTrue = 1;
            current = carMP2;
            currentWheel = carMP2;
        }
        else if (geom == carPlayer2->geoms[3] || geom == carPlayer2->geoms[1]) {
            CarLWTrue = 1;
            currentWheel = carMP2;
        }
        else {
            CarBTrue = 0;
            CarLWTrue = 0;
        }
        if (checkColliding(geom))
        {
            // hide non colliding geoms (car counter weights)
            drawGeomTWOP(geom, CarBTrue, CarLWTrue,current,currentWheel);
        }
    }
}

void drawAllSpaceCarSLGeoms(dSpaceID space, vehicle* carSL[])
{
    int ng = dSpaceGetNumGeoms(space);
    int CarBTrue = 0;
    int CarLWTrue = 0;
    CarModel current = Amuscle;
    CarModel currentWheel = Amuscle;
    for (int i = 0; i < ng; i++) {
        dGeomID geom = dSpaceGetGeom(space, i);
        if (geom == carSL[0]->geoms[0]) {
            CarBTrue = 1;
            current = Amuscle;
            currentWheel = Amuscle;
        }
        else if (geom == carSL[0]->geoms[3] || geom == carSL[0]->geoms[1]) {
            CarLWTrue = 1;
            currentWheel = Amuscle;
        }
        else if (geom == carSL[1]->geoms[0]) {
            CarBTrue = 1;
            current =MARUSSIA;
            currentWheel = MARUSSIA;
        }
        else if (geom == carSL[1]->geoms[3] || geom == carSL[1]->geoms[1]) {
            CarLWTrue = 1;
            currentWheel = MARUSSIA;
        }
        else if (geom == carSL[2]->geoms[0]) {
            CarBTrue = 1;
            current = BMW;
            currentWheel = BMW;
        }
        else if (geom == carSL[2]->geoms[3] || geom == carSL[2]->geoms[1]) {
            CarLWTrue = 1;
            currentWheel = BMW;
        }
        else {
            CarBTrue = 0;
            CarLWTrue = 0;
        }
        if (checkColliding(geom))
        {
            // hide non colliding geoms (car counter weights)
            drawGeomTWOP(geom, CarBTrue, CarLWTrue, current, currentWheel);
        }
    }
}


vehicle* CreateVehicle(dSpaceID space, dWorldID world,double posX, double posY, double posZ,double wheelOFF[])
{
    // TODO these should be parameters
    Vector3 carScale = (Vector3){ 4.6, 0.25, 1.8 };
    float wheelRadius = 0.5, wheelWidth = 0.6;

    vehicle* car = RL_MALLOC(sizeof(vehicle));
    
    //Rotation Initialization
    double rotini[16] = {-0.08,-0.02,1.00,0.00,0.05,1.00,0.02,0.00,-0.99,0.05,-0.08,0.00,0.68,0.01,0.74,0.01};
    dReal* RotIni = rotini;
    /*rotini[0] = -0.13;
    rotini[1] = -0.03;
    rotini[2] = 0.99;
    rotini[3] = 0.00;
    rotini[4] = 0.01;
    rotini[5] = 1.00;
    rotini[6] = 0.04;
    rotini[7] = 0.00;
    rotini[8] = -0.99;
    rotini[9] = 0.01;
    rotini[10] = -0.13;
    rotini[11] = 0.00;*/

    // car body
    dMass m;
    dMassSetBox(&m, 1, carScale.x, carScale.y, carScale.z);  // density
    dMassAdjust(&m, 800); // mass

    car->bodies[0] = dBodyCreate(world);
    dBodySetMass(car->bodies[0], &m);
    dBodySetAutoDisableFlag(car->bodies[0], 0);


    car->geoms[0] = dCreateBox(space, carScale.x, carScale.y, carScale.z);
    dGeomSetBody(car->geoms[0], car->bodies[0]);

    // TODO used a little later and should be a parameter
    dBodySetPosition(car->bodies[0], posX, posY, posZ);
    

    dGeomID front = dCreateBox(space, 2.3, 0.4, 1.6);
    dGeomSetBody(front, car->bodies[0]);
    dGeomSetOffsetPosition(front, -0.2, carScale.y, 0);

    car->bodies[5] = dBodyCreate(world);
    dBodySetMass(car->bodies[5], &m);
    dBodySetAutoDisableFlag(car->bodies[5], 0);
    // see previous TODO
    dBodySetPosition(car->bodies[5], posX, posY- 1, posZ);
    car->geoms[5] = dCreateSphere(space, 1);
    dGeomSetBody(car->geoms[5], car->bodies[5]);
    disabled.collidable = false;
    dGeomSetData(car->geoms[5], &disabled);

    car->joints[5] = dJointCreateFixed(world, 0);
    dJointAttach(car->joints[5], car->bodies[0], car->bodies[5]);
    dJointSetFixed(car->joints[5]);

    // wheels
    dMassSetCylinder(&m, 1, 3, wheelRadius, wheelWidth);
    dMassAdjust(&m, 60); // mass
    dQuaternion q;
    dQFromAxisAndAngle(q, 0, 0, 1, M_PI * 0.5);
    for (int i = 1; i <= 4; ++i)
    {
        car->bodies[i] = dBodyCreate(world);
        dBodySetMass(car->bodies[i], &m);
        dBodySetQuaternion(car->bodies[i], q);
        car->geoms[i] = dCreateCylinder(space, wheelRadius, wheelWidth);
        dGeomSetBody(car->geoms[i], car->bodies[i]);
        dBodySetFiniteRotationMode(car->bodies[i], 1);
        dBodySetAutoDisableFlag(car->bodies[i], 0);
    }

    const dReal* cp = dBodyGetPosition(car->bodies[0]);
    // TODO wheel base and axel width should be parameters
    dBodySetPosition(car->bodies[1], cp[0] + wheelOFF[0], cp[1] - wheelOFF[1], cp[2] - wheelOFF[2]);
    dBodySetPosition(car->bodies[2], cp[0] + wheelOFF[0], cp[1] - wheelOFF[1], cp[2] + wheelOFF[2]);
    dBodySetPosition(car->bodies[3], cp[0] - wheelOFF[3], cp[1] - wheelOFF[1], cp[2] - wheelOFF[2]);
    dBodySetPosition(car->bodies[4], cp[0] - wheelOFF[3], cp[1] - wheelOFF[1], cp[2] + wheelOFF[2]);

    // hinge2 (combined steering / suspension / motor !)
    for (int i = 0; i < 4; ++i)
    {
        car->joints[i] = dJointCreateHinge2(world, 0);
        dJointAttach(car->joints[i], car->bodies[0], car->bodies[i + 1]);
        const dReal* wPos = dBodyGetPosition(car->bodies[i + 1]);
        dJointSetHinge2Anchor(car->joints[i], wPos[0], wPos[1], wPos[2]);

        dReal axis1[] = { 0, -1, 0 };
        dReal axis2[] = { 0, 0, ((i % 2) == 0) ? -1 : 1 };

        // replacement for deprecated calls
        dJointSetHinge2Axes(car->joints[i], axis1, axis2);
        //dJointSetHinge2Axis1(joints[i], 0, 1, 0);
        //dJointSetHinge2Axis2(joints[i], 0, 0, ((i % 2) == 0) ? -1 : 1);

        dJointSetHinge2Param(car->joints[i], dParamLoStop, 0);
        dJointSetHinge2Param(car->joints[i], dParamHiStop, 0);
        dJointSetHinge2Param(car->joints[i], dParamLoStop, 0);
        dJointSetHinge2Param(car->joints[i], dParamHiStop, 0);
        dJointSetHinge2Param(car->joints[i], dParamFMax, 1500);

        dJointSetHinge2Param(car->joints[i], dParamVel2, dInfinity);
        dJointSetHinge2Param(car->joints[i], dParamFMax2, 1500);

        dJointSetHinge2Param(car->joints[i], dParamSuspensionERP, 0.7);
        dJointSetHinge2Param(car->joints[i], dParamSuspensionCFM, 0.001);//0.0025

        // steering
        if (i < 2) {
            dJointSetHinge2Param(car->joints[i], dParamFMax, 500);
            dJointSetHinge2Param(car->joints[i], dParamLoStop, -0.5);
            dJointSetHinge2Param(car->joints[i], dParamHiStop, 0.5);
            dJointSetHinge2Param(car->joints[i], dParamLoStop, -0.5);
            dJointSetHinge2Param(car->joints[i], dParamHiStop, 0.5);
            dJointSetHinge2Param(car->joints[i], dParamFudgeFactor, 0.1);
        }

    }
    // disable motor on front wheels
    dJointSetHinge2Param(car->joints[0], dParamFMax2, 0);
    dJointSetHinge2Param(car->joints[1], dParamFMax2, 0);
    dBodySetRotation(car->bodies[0], RotIni);

    return car;
}

void updateVehicle(vehicle* car, float accel, float maxAccelForce,
    float steer, float steerFactor)
{
    float target;
    target = 0;
    if (fabs(accel) > 0.1) target = maxAccelForce;
    //dJointSetHinge2Param( car->joints[0], dParamVel2, -accel );
    //dJointSetHinge2Param( car->joints[1], dParamVel2, accel );
    dJointSetHinge2Param(car->joints[2], dParamVel2, -accel);
    dJointSetHinge2Param(car->joints[3], dParamVel2, accel);

    //dJointSetHinge2Param( car->joints[0], dParamFMax2, target );
    //dJointSetHinge2Param( car->joints[1], dParamFMax2, target );
    dJointSetHinge2Param(car->joints[2], dParamFMax2, target);
    dJointSetHinge2Param(car->joints[3], dParamFMax2, target);
    for (int i = 0; i < 2; i++) {
        dReal v = steer - dJointGetHinge2Angle1(car->joints[i]);
        v *= steerFactor;
        dJointSetHinge2Param(car->joints[i], dParamVel, v);
    }
}

void unflipVehicle(vehicle* car)
{
    const dReal* cp = dBodyGetPosition(car->bodies[0]);
    dBodySetPosition(car->bodies[0], cp[0], cp[1] + 2, cp[2]);

    const dReal* R = dBodyGetRotation(car->bodies[0]);
    dReal newR[16];
    dRFromEulerAngles(newR, 0, -atan2(-R[2], R[0]), 0);
    dBodySetRotation(car->bodies[0], newR);

    // wheel offsets
    // TODO make configurable & use in vehicle set up 
    dReal wheelOffsets[4][3] = {
           { +1.2, -.6, -1 },
           { +1.2, -.6, +1 },
           { -1.2, -.6, -1 },
           { -1.2, -.6, +1 }
    };

    for (int i = 1; i < 5; i++) {
        dVector3 pb;
        dBodyGetRelPointPos(car->bodies[0], wheelOffsets[i - 1][0], wheelOffsets[i - 1][1], wheelOffsets[i - 1][2], pb);
        dBodySetPosition(car->bodies[i], pb[0], pb[1], pb[2]);
    }

}

// globals in use by near callback
dWorldID world;
dJointGroupID contactgroup;

Model box;
Model ball;
Model cylinder;
Model WheelAMDCR;
Model WheelAMDCL;
Model AM;
Model Marussia;
Model WheelMarussiaR;
Model WheelMarussiaL;
Model BMWi8;
Model WheelBMWi8R;
Model WheelBMWi8L;

int numObj = 1; // number of bodies


inline float rndf(float min, float max);
// macro candidate ? marcro's? eek!

float rndf(float min, float max)
{
    return ((float)rand() / (float)(RAND_MAX)) * (max - min) + min;
}



// when objects potentially collide this callback is called
// you can rule out certain collisions or use different surface parameters
// depending what object types collide.... lots of flexibility and power here!
#define MAX_CONTACTS 8

static void nearCallback(void* data, dGeomID o1, dGeomID o2)
{
    (void)data;
    int i;

    // exit without doing anything if the two bodies are connected by a joint
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    //if (b1==b2) return;
    if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact))
        return;

    if (!checkColliding(o1)) return;
    if (!checkColliding(o2)) return;

    // getting these just so can sometimes be a little bit of a black art!
    dContact contact[MAX_CONTACTS]; // up to MAX_CONTACTS contacts per body-body
    for (i = 0; i < MAX_CONTACTS; i++) {
        contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
            dContactSoftERP | dContactSoftCFM | dContactApprox1;
        contact[i].surface.mu = 1000;
        contact[i].surface.slip1 = 0.0001;
        contact[i].surface.slip2 = 0.001;
        contact[i].surface.soft_erp = 0.5;
        contact[i].surface.soft_cfm = 0.0003;

        contact[i].surface.bounce = 0.1;
        contact[i].surface.bounce_vel = 0.1;

    }
    int numc = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom,
        sizeof(dContact));
    if (numc) {
        dMatrix3 RI;
        dRSetIdentity(RI);
        for (i = 0; i < numc; i++) {
            dJointID c =
                dJointCreateContact(world, contactgroup, contact + i);
            dJointAttach(c, b1, b2);
        }
    }

}

static TextureCubemap GenTextureCubemap(Shader shader, Texture2D panorama, int size, int format) {
    TextureCubemap cubemap = { 0 };
    
    rlDisableBackfaceCulling();
    //Setup Frame Buffer
    unsigned int rbo = rlLoadTextureDepth(size, size, true);
    cubemap.id = rlLoadTextureCubemap(0, size, format);

    unsigned int fbo = rlLoadFramebuffer(size, size);
    rlFramebufferAttach(fbo, rbo, RL_ATTACHMENT_DEPTH, RL_ATTACHMENT_RENDERBUFFER, 0);
    rlFramebufferAttach(fbo, cubemap.id, RL_ATTACHMENT_COLOR_CHANNEL0, RL_ATTACHMENT_CUBEMAP_POSITIVE_X, 0);

    if (rlFramebufferComplete(fbo)) TraceLog(LOG_INFO, "FBO: [ID %i] Framebuffer object created successfully", fbo);

    //Draw to framebuffer

    rlEnableShader(shader.id);

    //Define projection matrix and send it to shader
    Matrix matFboProjection = MatrixPerspective(90.0 * DEG2RAD, 1.0, RL_CULL_DISTANCE_NEAR, RL_CULL_DISTANCE_FAR);
    rlSetUniformMatrix(shader.locs[SHADER_LOC_MATRIX_PROJECTION], matFboProjection);

    //Define view matrix for every side of cubemap
    Matrix fboViews[6] = {
        MatrixLookAt((Vector3) { 0.0f,0.0f,0.0f },(Vector3) { 1.0f,0.0f,0.0f },(Vector3) { 0.0f,-1.0f,0.0f }),
        MatrixLookAt((Vector3) { 0.0f,0.0f,0.0f },(Vector3) { -1.0f,0.0f,0.0f },(Vector3) { 0.0f,-1.0f,0.0f }),
        MatrixLookAt((Vector3) { 0.0f,0.0f,0.0f },(Vector3) { 0.0f,1.0f,0.0f },(Vector3) { 0.0f,0.0f,1.0f }),
        MatrixLookAt((Vector3) { 0.0f,0.0f,0.0f },(Vector3) { 0.0f,-1.0f,0.0f },(Vector3) { 0.0f,0.0f,-1.0f }),
        MatrixLookAt((Vector3) { 0.0f,0.0f,0.0f },(Vector3) { 0.0f,0.0f,1.0f },(Vector3) { 0.0f,-1.0f,0.0f }),
        MatrixLookAt((Vector3) { 0.0f,0.0f,0.0f },(Vector3) { 0.0f,0.0f,-1.0f },(Vector3) { 0.0f,-1.0f,0.0f })
    };

    rlViewport(0, 0, size, size);

    rlActiveTextureSlot(0);
    rlEnableTexture(panorama.id);

    for (int i = 0; i < 6; i++) {
        rlSetUniformMatrix(shader.locs[SHADER_LOC_MATRIX_VIEW], fboViews[i]);

        rlFramebufferAttach(fbo, cubemap.id, RL_ATTACHMENT_COLOR_CHANNEL0, RL_ATTACHMENT_CUBEMAP_POSITIVE_X + i, 0);
        rlEnableFramebuffer(fbo);

        rlClearScreenBuffers();
        rlLoadDrawCube();
    }

    rlDisableShader();
    rlDisableTexture();
    rlDisableFramebuffer();
    rlUnloadFramebuffer(fbo);

    rlViewport(0, 0, rlGetFramebufferWidth(), rlGetFramebufferHeight());
    rlEnableBackfaceCulling();

    cubemap.width = size;
    cubemap.height = size;
    cubemap.mipmaps = 1;
    cubemap.format = format;

    return cubemap;


}


//AI acceleration
int IsAIAccel(Vector3 aico,Vector3 aidir,Vector3 wpts[]) {
    float dist[44];
    float mind;
    int minwaypoint;
    Vector3 waypoint[44];

    /*Vector3 waypoints[174];
    FILE* fwpts;
    fwpts = fopen("waypoints.txt", "r");
    for (int i = 0; i < 174; i++) {
        float x, y, z;
        char s[50];
        fgets(s, 50, fwpts);
        sscanf(s, "%f %f %f", &x, &y, &z);
        waypoints[i] = (Vector3){ x,y,z };
    }*/
    /*
    waypoint[0]= (Vector3){ 118.719,-6.218,173.879 };
    waypoint[1] = (Vector3){ 115.051,-6.186,147.493 };
    waypoint[2] = (Vector3){ 111.146,-5.872,128.098 };
    waypoint[3] = (Vector3){ 106.872,-5.083,106.504 };
    waypoint[4] = (Vector3){ 104.613,-3.636,80.435 };
    waypoint[5] = (Vector3){ 103.491,-2.670,61.030 };
    waypoint[6] = (Vector3){ 100.436,-1.904,34.146 };
    waypoint[7] = (Vector3){ 80.396,-1.856,18.974 };
    waypoint[8] = (Vector3){ 55.559,-2.151,6.571 };
    waypoint[9] = (Vector3){ 31.624,-2.122,-20.370};
    waypoint[10] = (Vector3){ 29.035,-1.810,-45.561 };
    waypoint[11] = (Vector3){ 35.812,0.540,-88.303 };
    waypoint[12] = (Vector3){ 57.248,2.751,-115.669 };
    waypoint[13] = (Vector3){ 75.400,3.899,-129.362 };
    waypoint[14] = (Vector3){ 97.722,5.678,-148.041 };
    waypoint[15] = (Vector3){ 126.368,7.613,-170.657 };
    waypoint[16] = (Vector3){ 121.647,7.573,-187.444 };
    waypoint[17] = (Vector3){ 99.010,6.477,-193.176 };
    waypoint[18] = (Vector3){ 58.772,4.898,-197.537 };
    waypoint[19] = (Vector3){ 34.417,4.108,-193.334 };
    waypoint[20] = (Vector3){ 3.746,3.177,-181.651 };
    waypoint[21] = (Vector3){ -27.108,2.468,-174.133 };
    waypoint[22] = (Vector3){ -50.096,1.852,-169.013};
    waypoint[23] = (Vector3){ -65.686,0.877,-153.801 };
    waypoint[24] = (Vector3){ -51.318,0.586,-146.536 };
    waypoint[25] = (Vector3){ -26.070,0.782,-134.494 };
    waypoint[26] = (Vector3){ -26.404,-0.099,-101.459};
    waypoint[27] = (Vector3){ -36.324,-3.042,-62.711 };
    waypoint[28] = (Vector3){ -43.081,-5.498,-21.555 };
    waypoint[29] = (Vector3){ -47.949,-5.682,6.331 };
    waypoint[30] = (Vector3){ -54.335,-6.150,34.994};
    waypoint[31] = (Vector3){ -59.149,-6.777,62.021};
    waypoint[32] = (Vector3){ -62.647,-7.575,94.978 };
    waypoint[33] = (Vector3){ -60.971,-8.446,122.479 };
    waypoint[34] = (Vector3){ -52.048,-9.047,148.822 };
    waypoint[35] = (Vector3){ -32.233,-9.072,168.424 };
    waypoint[36] = (Vector3){ -14.878,-9.290,185.018 };
    waypoint[37] = (Vector3){ 6.265,-9.513,200.361 };
    waypoint[38] = (Vector3){ 31.164,-9.459,202.823 };
    waypoint[39] = (Vector3){ 49.551,-9.179,193.120 };
    waypoint[40] = (Vector3){ 64.950,-8.514,173.876 };
    waypoint[41] = (Vector3){ 76.390,-7.420,152.000 };
    waypoint[42] = (Vector3){ 86.826,-5.937,120.567 };
    waypoint[43] = (Vector3){ 94.833,-4.652,97.582 };*/

    for (int i = 0; i < 174; i++) {
        dist[i] = Vector3Distance(aico, wpts[i]);
        if (i == 0) {
            mind = dist[i];
            minwaypoint = 0;
        }
        if (mind > dist[i]) {
            mind = dist[i];
            minwaypoint = i;
        }
    }
    //fclose(fwpts);
    printf("%.2f %.2f %.2f\n", wpts[minwaypoint].x, wpts[minwaypoint].y, wpts[minwaypoint].z);
    printf("%d\n", minwaypoint);
    if ((Vector3DotProduct(aidir, Vector3Subtract(Vector3Lerp(aico, wpts[minwaypoint],0.4),aico)) >= 0)&& mind >= 2) {
        return 1;
    }
    else if ((Vector3DotProduct(aidir, Vector3Subtract(Vector3Lerp(aico, wpts[minwaypoint], 0.4), aico)) < 0)&& mind>=6) {
        return -1;
    }
}

float  AISteer(Vector3 aico, Vector3 aidir,Vector3 wpts[]) {
    float dist[44];
    float mind;
    int minwaypoint;
    Vector3 waypoint[44];
    float angleToDir;
    float dotp;

    Vector3 waypoints[174];
    

    for (int i = 0; i < 174; i++) {
        dist[i] = Vector3Distance(aico, wpts[i]);
        if (i == 0) {
            mind = dist[i];
            minwaypoint = 0;
        }
        if (mind > dist[i]) {
            mind = dist[i];
            minwaypoint = i;
        }
    }
    //fclose(fwpts);
    angleToDir = Vector3Angle(aidir, Vector3Subtract(Vector3Lerp(aico, wpts[minwaypoint], 0.4),aico)).x;
    dotp = Vector3DotProduct(Vector3Normalize(aidir),Vector3Normalize(Vector3Subtract(Vector3Lerp(aico, wpts[minwaypoint], 0.4), aico)));
    if (Vector3DotProduct(aidir, Vector3Subtract(Vector3Lerp(aico, wpts[minwaypoint], 0.4), aico)) < 0||mind<=2.0f) {
        angleToDir = Vector3Angle(aidir, Vector3Subtract(Vector3Lerp(aico, wpts[minwaypoint+1], 0.4), aico)).x;
        dotp = Vector3DotProduct(Vector3Normalize(aidir),Vector3Normalize(Vector3Subtract(Vector3Lerp(aico, wpts[minwaypoint + 1], 0.4), aico)));
    }
    /*if (angleToDir > 0) {
        return angleToDir*RAD2DEG/90;
    }
    else if (angleToDir < 0 ) {
        return angleToDir * RAD2DEG / 90;
    }*/
    if (dotp > 0) {
        return dotp;
    }
    else if (dotp < 0) {
        return dotp;
    }

}

typedef enum GameScreen {
    SPLASH_RL, SPLASH_ODE, MENU, STORYMODE,CARSL,RACEMODESPLIT,RACEMODEAI, FREERIDE, OPTIONS
} GameScreen;
//typedef enum MenuScreen {STORYMODE,RACEMODE,FREERIDE,OPTIONS} MenuScreen;

void DrawCarModelSL(Model carM, Model whL, Model whR,Vector3 carPos,double whoff[]) {
    DrawModel(carM, carPos, 1.0f, WHITE);
    DrawModelEx(whL, (Vector3) { carPos.x + whoff[0], carPos.y - whoff[1], carPos.z - whoff[2] }, Vector3Zero(), 0, (Vector3) { 1.0f, 1.0f, 0.6f }, WHITE);
    DrawModelEx(whR, (Vector3) { carPos.x + whoff[0], carPos.y - whoff[1], carPos.z + whoff[2] }, Vector3Zero(), 0, (Vector3) { 1.0f, 1.0f, 0.6f }, WHITE);
    DrawModelEx(whL, (Vector3) { carPos.x - whoff[3], carPos.y - whoff[1], carPos.z - whoff[2] }, Vector3Zero(), 0, (Vector3) { 1.0f, 1.0f, 0.6f }, WHITE);
    DrawModelEx(whR, (Vector3) { carPos.x - whoff[3], carPos.y - whoff[1], carPos.z + whoff[2] }, Vector3Zero(), 0, (Vector3) { 1.0f, 1.0f, 0.6f }, WHITE);
}

int CheckPointIndex(dReal* carpos,Vector3 chpts[],int prevChIndex) {
    if (Vector3Distance(chpts[prevChIndex + 1], (Vector3) { carpos[0], carpos[1], carpos[2] }) <= 6.0f) {
        return prevChIndex + 1;
    }
    else {
        return prevChIndex;
    }
}

int main(void)
{
    
    const int screenWidth = 1200;
    const int screenHeight = 720;

    assert(sizeof(dReal) == sizeof(double));
    srand(time(NULL));

    GameScreen currentScreen = SPLASH_RL;
    GameScreen nextScreen = MENU;
    //MenuScreen cMainScreen = FREERIDE;

    dSpaceID space;
    dSpaceID spaceAI;
    dSpaceID space2P;
    dSpaceID spaceCARSL;

    dBodyID obj[numObj];

    SetWindowState(FLAG_VSYNC_HINT | FLAG_MSAA_4X_HINT );
    InitWindow(screenWidth, screenHeight, "Ride Again");
    //ToggleFullscreen();

    InitAudioDevice();
    
    Camera camera = { (Vector3) { 25.0f, 15.0f, 25.0f }, (Vector3) { 0.0f, 0.5f, 0.0f },
                        (Vector3) {0.0f, 1.0f, 0.0f}, 45.0f, CAMERA_PERSPECTIVE };
    Camera cameraPlayer1 = { (Vector3) { 25.0f, 15.0f, 25.0f }, (Vector3) { 0.0f, 0.5f, 0.0f },
                        (Vector3) {0.0f, 1.0f, 0.0f}, 45.0f, CAMERA_PERSPECTIVE };
    Camera cameraPlayer2 = { (Vector3) { 25.0f, 15.0f, 25.0f }, (Vector3) { 0.0f, 0.5f, 0.0f },
                        (Vector3) {0.0f, 1.0f, 0.0f}, 45.0f, CAMERA_PERSPECTIVE };
    Camera cameraCARSLSPLIT= { (Vector3) { 25.0f, 15.0f, 25.0f }, (Vector3) { 0.0f, 0.5f, 0.0f },
                        (Vector3) {0.0f, 1.0f, 0.0f}, 45.0f, CAMERA_PERSPECTIVE };
    

    RenderTexture screenPlayer1 = LoadRenderTexture(screenWidth / 2, screenHeight);
    RenderTexture screenPlayer2 = LoadRenderTexture(screenWidth / 2, screenHeight);
    Rectangle splitScreenRect = { 0.0f,0.0f,(float)screenPlayer1.texture.width,(float)-screenPlayer1.texture.height };

    box = LoadModelFromMesh(GenMeshCube(1, 1, 1));
    ball = LoadModelFromMesh(GenMeshSphere(.5, 32, 32));
    cylinder = LoadModel("Models/data/cylinder.obj");
    WheelMarussiaR = LoadModel("ModelArchives/CarMarussia/MarussiaWheelWR.obj");
    WheelMarussiaL = LoadModel("ModelArchives/CarMarussia/MarussiaWheelWL.obj");
    Marussia = LoadModel("ModelArchives/CarMarussia/MarussiaBodyF.obj");
    WheelAMDCR = LoadModel("ModelArchives/CarAmericanMuscle/AmericanMuscleLP.obj");
    WheelAMDCL = LoadModel("ModelArchives/CarAmericanMuscle/AmericanMuscleWL_LP.obj");
    AM = LoadModel("ModelArchives/CarAmericanMuscle/AmericanMuscleLP12.obj");
    BMWi8 = LoadModel("ModelArchives/CarBMWi8/BMWi8Body.obj");
    WheelBMWi8L = LoadModel("ModelArchives/CarBMWi8/BMWi8WL.obj");
    WheelBMWi8R= LoadModel("ModelArchives/CarBMWi8/BMWi8WR.obj");
    

    CarModel carMPlayer1 = BMW;
    CarModel carMPlayer2 = MARUSSIA;

    Model ground = LoadModel("ModelArchives/WorldIsraelRace/source/IsraelRace.obj");
    //Model ground = LoadModel("Models/data/ground.obj");
    Mesh skycube = GenMeshCube(2.0f, 2.0f, 2.0f);
    Model skybox = LoadModelFromMesh(skycube);

    Texture earthTx = LoadTexture("Models/data/earth.png");
    Texture crateTx = LoadTexture("Models/data/crate.png");
    Texture drumTx = LoadTexture("Models/data/drum.png");
    Texture grassTx = LoadTexture("ModelArchives/WorldIsraelRace/source/pazael_030319_lowqulity_texture.png");
    //Texture carMetal = LoadTexture("ModelArchives/CarAmericanMuscle/AmericanMuscle/source/textures/worn-shiny-metal-Metallic.png");
    //Texture carRough = LoadTexture("ModelArchives/CarAmericanMuscle/AmericanMuscle/source/textures/worn-shiny-metal-Roughness.png");

    //Skybox
    skybox.materials[0].shader = LoadShader(TextFormat("shaders/skybox.vs", GLSL_VERSION),
        TextFormat("shaders/skybox.fs", GLSL_VERSION));
    SetShaderValue(skybox.materials[0].shader, GetShaderLocation(skybox.materials[0].shader, "environmentMap"), (int[1]) { MATERIAL_MAP_CUBEMAP }, SHADER_UNIFORM_INT);
    SetShaderValue(skybox.materials[0].shader, GetShaderLocation(skybox.materials[0].shader, "doGamma"), (int[1]) { 1 ? 1 : 0 }, SHADER_UNIFORM_INT);
    SetShaderValue(skybox.materials[0].shader, GetShaderLocation(skybox.materials[0].shader, "vflipped"), (int[1]) { 1 ? 1 : 0 }, SHADER_UNIFORM_INT);

    // Load cubemap shader and setup required shader locations
    Shader shdrCubemap = LoadShader(TextFormat("shaders/cubemap.vs", GLSL_VERSION),
        TextFormat("shaders/cubemap.fs", GLSL_VERSION));

    SetShaderValue(shdrCubemap, GetShaderLocation(shdrCubemap, "equirectangularMap"), (int[1]) { 0 }, SHADER_UNIFORM_INT);

    Texture2D panorama;
    panorama = LoadTexture("textures/skybox/kloofendal_48d_partly_cloudy_4k.hdr");
    skybox.materials[0].maps[MATERIAL_MAP_CUBEMAP].texture = GenTextureCubemap(shdrCubemap, panorama, 1024, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8);

    //MainMenu
    Image MainBG[24] = { 0 };
    Image MainT[5] = { 0 };
    Image SplashR[51] = { 0 };
    Image SplashGT[71] = { 0 };
    char menuBG[][45] = { "textures/MainMenuBG/frame_00_delay-0.03s.png","textures/MainMenuBG/frame_01_delay-0.03s.png","textures/MainMenuBG/frame_02_delay-0.03s.png",
            "textures/MainMenuBG/frame_03_delay-0.03s.png","textures/MainMenuBG/frame_04_delay-0.03s.png","textures/MainMenuBG/frame_05_delay-0.03s.png" ,
            "textures/MainMenuBG/frame_06_delay-0.03s.png","textures/MainMenuBG/frame_07_delay-0.03s.png","textures/MainMenuBG/frame_08_delay-0.03s.png" ,
            "textures/MainMenuBG/frame_09_delay-0.03s.png","textures/MainMenuBG/frame_10_delay-0.03s.png","textures/MainMenuBG/frame_11_delay-0.03s.png",
            "textures/MainMenuBG/frame_12_delay-0.03s.png","textures/MainMenuBG/frame_13_delay-0.03s.png" ,"textures/MainMenuBG/frame_14_delay-0.03s.png",
            "textures/MainMenuBG/frame_15_delay-0.03s.png","textures/MainMenuBG/frame_16_delay-0.03s.png","textures/MainMenuBG/frame_17_delay-0.03s.png",
            "textures/MainMenuBG/frame_18_delay-0.03s.png","textures/MainMenuBG/frame_19_delay-0.03s.png","textures/MainMenuBG/frame_20_delay-0.03s.png",
            "textures/MainMenuBG/frame_21_delay-0.03s.png" ,"textures/MainMenuBG/frame_22_delay-0.03s.png" ,"textures/MainMenuBG/frame_23_delay-0.03s.png" ,
            "textures/MainMenuBG/frame_24_delay-0.03s.png" };
    char menuT[][43] = { "textures/MainMenuT/frame_0_delay-0.08s.png","textures/MainMenuT/frame_1_delay-0.08s.png","textures/MainMenuT/frame_2_delay-0.08s.png" ,
            "textures/MainMenuT/frame_3_delay-0.08s.png","textures/MainMenuT/frame_4_delay-0.08s.png" };
    char splashray[][39] = { "textures/SplashRay/ezgif-frame-001.png","textures/SplashRay/ezgif-frame-002.png","textures/SplashRay/ezgif-frame-003.png",
    "textures/SplashRay/ezgif-frame-004.png", "textures/SplashRay/ezgif-frame-005.png", "textures/SplashRay/ezgif-frame-006.png",
    "textures/SplashRay/ezgif-frame-007.png", "textures/SplashRay/ezgif-frame-008.png", "textures/SplashRay/ezgif-frame-009.png",
    "textures/SplashRay/ezgif-frame-010.png", "textures/SplashRay/ezgif-frame-011.png", "textures/SplashRay/ezgif-frame-012.png",
    "textures/SplashRay/ezgif-frame-013.png", "textures/SplashRay/ezgif-frame-014.png", "textures/SplashRay/ezgif-frame-015.png",
    "textures/SplashRay/ezgif-frame-016.png", "textures/SplashRay/ezgif-frame-017.png", "textures/SplashRay/ezgif-frame-018.png",
    "textures/SplashRay/ezgif-frame-019.png", "textures/SplashRay/ezgif-frame-020.png", "textures/SplashRay/ezgif-frame-021.png",
    "textures/SplashRay/ezgif-frame-022.png", "textures/SplashRay/ezgif-frame-023.png", "textures/SplashRay/ezgif-frame-024.png",
    "textures/SplashRay/ezgif-frame-025.png", "textures/SplashRay/ezgif-frame-026.png", "textures/SplashRay/ezgif-frame-027.png",
    "textures/SplashRay/ezgif-frame-028.png", "textures/SplashRay/ezgif-frame-029.png", "textures/SplashRay/ezgif-frame-030.png",
    "textures/SplashRay/ezgif-frame-031.png", "textures/SplashRay/ezgif-frame-032.png", "textures/SplashRay/ezgif-frame-033.png",
    "textures/SplashRay/ezgif-frame-034.png", "textures/SplashRay/ezgif-frame-035.png", "textures/SplashRay/ezgif-frame-036.png",
    "textures/SplashRay/ezgif-frame-037.png", "textures/SplashRay/ezgif-frame-038.png", "textures/SplashRay/ezgif-frame-039.png",
    "textures/SplashRay/ezgif-frame-040.png", "textures/SplashRay/ezgif-frame-041.png", "textures/SplashRay/ezgif-frame-042.png",
    "textures/SplashRay/ezgif-frame-043.png", "textures/SplashRay/ezgif-frame-044.png", "textures/SplashRay/ezgif-frame-045.png",
    "textures/SplashRay/ezgif-frame-046.png", "textures/SplashRay/ezgif-frame-047.png", "textures/SplashRay/ezgif-frame-048.png",
    "textures/SplashRay/ezgif-frame-049.png", "textures/SplashRay/ezgif-frame-050.png", "textures/SplashRay/ezgif-frame-051.png" };
    char splashgt[][39] = { "textures/SplashGT/ezgif-frame-001.png","textures/SplashGT/ezgif-frame-002.png","textures/SplashGT/ezgif-frame-003.png",
    "textures/SplashGT/ezgif-frame-004.png", "textures/SplashGT/ezgif-frame-005.png", "textures/SplashGT/ezgif-frame-006.png",
    "textures/SplashGT/ezgif-frame-007.png", "textures/SplashGT/ezgif-frame-008.png", "textures/SplashGT/ezgif-frame-009.png",
    "textures/SplashGT/ezgif-frame-010.png", "textures/SplashGT/ezgif-frame-011.png", "textures/SplashGT/ezgif-frame-012.png",
    "textures/SplashGT/ezgif-frame-013.png", "textures/SplashGT/ezgif-frame-014.png", "textures/SplashGT/ezgif-frame-015.png",
    "textures/SplashGT/ezgif-frame-016.png", "textures/SplashGT/ezgif-frame-017.png", "textures/SplashGT/ezgif-frame-018.png",
    "textures/SplashGT/ezgif-frame-019.png", "textures/SplashGT/ezgif-frame-020.png", "textures/SplashGT/ezgif-frame-021.png",
    "textures/SplashGT/ezgif-frame-022.png", "textures/SplashGT/ezgif-frame-023.png", "textures/SplashGT/ezgif-frame-024.png",
    "textures/SplashGT/ezgif-frame-025.png", "textures/SplashGT/ezgif-frame-026.png", "textures/SplashGT/ezgif-frame-027.png",
    "textures/SplashGT/ezgif-frame-028.png", "textures/SplashGT/ezgif-frame-029.png", "textures/SplashGT/ezgif-frame-030.png",
    "textures/SplashGT/ezgif-frame-031.png", "textures/SplashGT/ezgif-frame-032.png", "textures/SplashGT/ezgif-frame-033.png",
    "textures/SplashGT/ezgif-frame-034.png", "textures/SplashGT/ezgif-frame-035.png", "textures/SplashGT/ezgif-frame-036.png",
    "textures/SplashGT/ezgif-frame-037.png", "textures/SplashGT/ezgif-frame-038.png", "textures/SplashGT/ezgif-frame-039.png",
    "textures/SplashGT/ezgif-frame-040.png", "textures/SplashGT/ezgif-frame-041.png", "textures/SplashGT/ezgif-frame-042.png",
    "textures/SplashGT/ezgif-frame-043.png", "textures/SplashGT/ezgif-frame-044.png", "textures/SplashGT/ezgif-frame-045.png",
    "textures/SplashGT/ezgif-frame-046.png", "textures/SplashGT/ezgif-frame-047.png", "textures/SplashGT/ezgif-frame-048.png",
    "textures/SplashGT/ezgif-frame-049.png", "textures/SplashGT/ezgif-frame-050.png", "textures/SplashGT/ezgif-frame-051.png",
    "textures/SplashGT/ezgif-frame-052.png", "textures/SplashGT/ezgif-frame-053.png", "textures/SplashGT/ezgif-frame-054.png", 
    "textures/SplashGT/ezgif-frame-055.png", "textures/SplashGT/ezgif-frame-056.png", "textures/SplashGT/ezgif-frame-057.png", 
    "textures/SplashGT/ezgif-frame-058.png", "textures/SplashGT/ezgif-frame-059.png", "textures/SplashGT/ezgif-frame-060.png",
    "textures/SplashGT/ezgif-frame-061.png", "textures/SplashGT/ezgif-frame-062.png", "textures/SplashGT/ezgif-frame-063.png", 
    "textures/SplashGT/ezgif-frame-064.png", "textures/SplashGT/ezgif-frame-065.png", "textures/SplashGT/ezgif-frame-066.png",
    "textures/SplashGT/ezgif-frame-067.png", "textures/SplashGT/ezgif-frame-068.png", "textures/SplashGT/ezgif-frame-069.png",
    "textures/SplashGT/ezgif-frame-070.png", "textures/SplashGT/ezgif-frame-071.png" };
    Texture2D menutexBG[24] = { 0 };
    Texture2D menutexT[5] = { 0 };
    Texture2D SplashRaytex[51] = { 0 };
    Texture2D SplashGTtex[71] = { 0 };
    for (int i = 0; i < 71; i++) {
        SplashGT[i] = LoadImage(splashgt + i);
        SplashGTtex[i] = LoadTextureFromImage(SplashGT[i]);
        UnloadImage(SplashGT[i]);
    }
    for (int i = 0; i < 51;i++) {
            SplashR[i] = LoadImage(splashray + i);
            SplashRaytex[i] = LoadTextureFromImage(SplashR[i]);
            UnloadImage(SplashR[i]);
    }
    for (int i = 0; i < 24;i++) {
            MainBG[i] = LoadImage(menuBG + i);
            menutexBG[i] = LoadTextureFromImage(MainBG[i]);
            UnloadImage(MainBG[i]);
     }
    for (int i = 0; i < 5;i++) {
            MainT[i] = LoadImage(*(menuT + i));
            menutexT[i] = LoadTextureFromImage(MainT[i]);
            UnloadImage(MainT[i]);
    }
    

    //Sound and Music
    Music music = LoadMusicStream("audio/CrockettsTheme.mp3");
    Music splashRM = LoadMusicStream("audio/QuickLightStreakLogo_free.mp3");
    Music splashGTM = LoadMusicStream("audio/IgnitingLogoReveal_free.mp3");
    Sound fxWav = LoadSound("audio/sound.wav");         
    Sound fxOgg = LoadSound("audio/target.ogg");
    float pitch = 1.0f;
    

    //PlayMusicStream(music);

    float timePlayed = 0.0f;
    bool pause = false;

    box.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = crateTx;
    ball.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = earthTx;
    cylinder.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = drumTx;
    ground.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = grassTx;
    /*for (int i = 0; i < 7; i++) {
        AM.materials[i].maps[MATERIAL_MAP_METALNESS].texture = carMetal;
        AM.materials[i].maps[MATERIAL_MAP_ROUGHNESS].texture = carRough;
    }*/

    Shader shader = LoadShader("Models/data/simpleLight.vs", "Models/data/simpleLight.fs");
    Shader alphadiscard = LoadShader(NULL, "shaders/alpha_discard.fs");
    Shader l_shader = LoadShader(TextFormat("shaders/base_lighting.vs", GLSL_VERSION), TextFormat("shaders/lighting.fs", GLSL_VERSION));
    l_shader.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(l_shader, "viewPos");
    int ambientLoc = GetShaderLocation(l_shader, "ambient");
    SetShaderValue(l_shader, ambientLoc, (float[4]) { 0.1f, 0.1f, 0.1f, 1.0f }, RL_SHADER_UNIFORM_VEC4);

    shader.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocation(shader, "matModel");
    shader.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(shader, "viewPos");

    int amb = GetShaderLocation(shader, "ambient");
    SetShaderValue(shader, amb, (float[4]) { 0.2, 0.2, 0.2, 1.0 }, SHADER_UNIFORM_VEC4);

    box.materials[0].shader = shader;
    ball.materials[0].shader = shader;
    cylinder.materials[0].shader = shader;
    ground.materials[0].shader = shader;
    WheelAMDCR.materials[0].shader = shader;
    WheelAMDCL.materials[0].shader = shader;
    for (int i = 0; i < 7; i++) {
        AM.materials[i].shader = shader;
    }
    for (int i = 0; i < 6; i++) {
        Marussia.materials[i].shader = shader;
    }
    for (int i = 0; i < 4; i++) {
        WheelMarussiaL.materials[i].shader = shader;
        WheelMarussiaR.materials[i].shader = shader;
    }
    for (int i = 0; i < 6; i++) {
        BMWi8.materials[i].shader = shader;
    }
    for (int i = 0; i < 5; i++) {
        WheelBMWi8L.materials[i].shader = shader;
        WheelBMWi8R.materials[i].shader = shader;
    }

    Light lights[5];

    lights[0] = CreateLight(LIGHT_POINT, (Vector3) { -25, 7, 25 }, Vector3Zero(),
        (Color) {
        128, 128, 128, 255
    }, shader);
    lights[1] = CreateLight(LIGHT_POINT, (Vector3) { -25, 7, -25 }, Vector3Zero(),
        (Color) {
        64, 64, 64, 255
    }, shader);
    lights[2]= CreateLight(LIGHT_POINT, (Vector3) { -25, 7, 0 }, Vector3Zero(),
        (Color) {
        64, 64, 64, 255
    }, shader);
    lights[3]= CreateLight(LIGHT_POINT, (Vector3) { 117, 7, 156 }, Vector3Zero(),(Color) {64, 64, 64, 255}, shader);
    //lights[2] = CreateLight(LIGHT_DIRECTIONAL, (Vector3) { -25, 15, 20 }, Vector3Zero(), RED, shader);
    //lights[3]= CreateLight(LIGHT_POINT, (Vector3) { -25, 15, 20 }, Vector3Zero(), RED, shader);
   
    dInitODE2(0);   // initialise and create the physics
    dAllocateODEDataForThread(dAllocateMaskAll);

    world = dWorldCreate();
    printf("phys iterations per step %i\n", dWorldGetQuickStepNumIterations(world));
    space = dHashSpaceCreate(NULL);
    contactgroup = dJointGroupCreate(0);
    dWorldSetGravity(world, 0, -9.8, 0);    // gravity

    dWorldSetAutoDisableFlag(world, 1);
    dWorldSetAutoDisableLinearThreshold(world, 0.05);
    dWorldSetAutoDisableAngularThreshold(world, 0.05);
    dWorldSetAutoDisableSteps(world, 4);

    //2-Player Space
    space2P = dHashSpaceCreate(NULL);
    
    //AIMode Space
    spaceAI = dHashSpaceCreate(NULL);
    spaceCARSL= dHashSpaceCreate(NULL);

    vehicle* car;
    vehicle* AIcar;
    vehicle* carPlayer1;
    vehicle* carPlayer2;
    vehicle* carAIM;
    vehicle* carSLsc[3];
    
    // create some decidedly sub optimal indices!
    // for the ground trimesh
    int nV = ground.meshes[0].vertexCount;
    int* groundInd = RL_MALLOC(nV * sizeof(int));
    for (int i = 0; i < nV; i++) {
        groundInd[i] = i;
    }

    // static tri mesh data to geom
    dTriMeshDataID triData = dGeomTriMeshDataCreate();
    dGeomTriMeshDataBuildSingle(triData, ground.meshes[0].vertices,
        3 * sizeof(float), nV,
        groundInd, nV,
        3 * sizeof(int));
    dCreateTriMesh(space, triData, NULL, NULL, NULL);
    dCreateTriMesh(space2P, triData, NULL, NULL, NULL);
    dCreateTriMesh(spaceAI, triData, NULL, NULL, NULL);
    dCreateTriMesh(spaceCARSL, triData, NULL, NULL, NULL);
    


    // create the physics bodies
    for (int i = 0; i < numObj; i++) {
        obj[i] = dBodyCreate(world);
        dGeomID geom;
        dMatrix3 R;
        dMass m;
        float typ = rndf(0, 1);
        if (typ < .25) {                //  box
            Vector3 s = (Vector3){ rndf(0.5, 2), rndf(0.5, 2), rndf(0.5, 2) };
            geom = dCreateBox(space, s.x, s.y, s.z);
            dMassSetBox(&m, 1, s.x, s.y, s.z);
        }
        else if (typ < .5) {          //  sphere
            float r = rndf(0.5, 1);
            geom = dCreateSphere(space, r);
            dMassSetSphere(&m, 1, r);
        }
        else if (typ < .75) {         //  cylinder
            float l = rndf(0.5, 2);
            float r = rndf(0.25, 1);
            geom = dCreateCylinder(space, r, l);
            dMassSetCylinder(&m, 1, 3, r, l);
        }
        else {                        //  composite of cylinder with 2 spheres
            float l = rndf(.9, 1.5);

            geom = dCreateCylinder(space, 0.25, l);
            dGeomID geom2 = dCreateSphere(space, l / 2);
            dGeomID geom3 = dCreateSphere(space, l / 2);


            dMass m2, m3;
            dMassSetSphere(&m2, 1, l / 2);
            dMassTranslate(&m2, 0, 0, l - 0.25);
            dMassSetSphere(&m3, 1, l / 2);
            dMassTranslate(&m3, 0, 0, -l + 0.25);
            dMassSetCylinder(&m, 1, 3, .25, l);
            dMassAdd(&m2, &m3);
            dMassAdd(&m, &m2);

            dGeomSetBody(geom2, obj[i]);
            dGeomSetBody(geom3, obj[i]);
            dGeomSetOffsetPosition(geom2, 0, 0, l - 0.25);
            dGeomSetOffsetPosition(geom3, 0, 0, -l + 0.25);

        }

        // give the body a random position and rotation
        dBodySetPosition(obj[i],
            dRandReal() * 10 - 5, 4 + (i / 10), dRandReal() * 10 - 5);
        dRFromAxisAndAngle(R, dRandReal() * 2.0 - 1.0,
            dRandReal() * 2.0 - 1.0,
            dRandReal() * 2.0 - 1.0,
            dRandReal() * M_PI * 2 - M_PI);
        dBodySetRotation(obj[i], R);
        // set the bodies mass and the newly created geometry
        dGeomSetBody(geom, obj[i]);
        dBodySetMass(obj[i], &m);


    }


    float accel = 0, steer = 0;
    float accelP1 = 0, steerP1 = 0;
    float accelP2 = 0, steerP2 = 0;
    Vector3 debug = { 0 };
    Vector3 AMpos = { 117.161,-6.6,156.169 };
    Vector3 Marussiapos = { 115.161,-6.55,145.169 };
    Vector3 BMWi8pos = { 114.161,-6.45,134.169 };
    Vector3 currentCar=AMpos;
    bool antiSway = true;

    // keep the physics fixed time in step with the render frame
    // rate which we don't know in advance
    double frameTime = 0;
    double physTime = 0;
    const double physSlice = 1.0 / 240.0;
    const int maxPsteps = 6;
    int carFlipped = 0;
    int carFlippedAI = 0;
    int carP1Flipped = 0;
    int carP2Flipped = 0;
    Vector2 mouseDelta;
    Vector3 mousePos;
    Vector2 mousePoint = { 0.0f,0.0f };
    float yaw=-90.0f;
    float yawP1 = -90.0f;
    float yawP2 = -90.0f;
    float pitchcar=0.0f;
    float pitchcarP1 = 0.0f;
    float pitchcarP2 = 0.0f;
    float sensitivity = 0.2f;
    float campos[3];
    float camposd[3] = {0.0f,0.0f,0.0f};
    float camrot = 0.9;
    float speedoac,speedov;
    float speedoac1, speedov1, speedoac2, speedov2;
    float AIaccel = 15,AIsteer=0;
    int nBG=0;
    int nmT = 0;
    int nSplhR = 0;
    int nSplGT = 0;
    int freerideBtnState = 0, freerideBtnAction = 0, mainmenuBGM = 0;
    int menuFRback = 0,menuRM2Pback=0,menuRMAIback=0;
    int IsAICarCreated = 0;
    int IsP1CarCreated = 0;
    int IsP2CarCreated = 0;
    int IsRMCarCreatedAIM = 0;
    int IsRMCarCreatedFRM = 0;
    int IsCRSLAMCarCreated = 0;
    int IsFrame = 0;
    int IsP1CarSL = 0;
    int carP1chptIndex = -1;
    int carP2chptIndex = -1;
    int carwin2P = -1;
    int AIAccelFunc;
    float AISteerFunc;
    float AICarSteerSens = 0.09;
    int AIInitAccel = 0;
    double AMwheeloff[4] = { 1.6,0.5,0.9,1.15 };//{front,up,left/right,back}
    double AMwheeloffSL[4] = { 1.6,0.15,0.9,1.15 };
    double Marussiawheeloff[4]= { 1.5,0.5,0.9,1.45 };
    double MarussiawheeloffSL[4] = { 1.5,0.15,0.9,1.45 };
    double BMWi8wheeloff[4] = { 1.6,0.5,0.85,1.40 };
    double BMWi8wheeloffSL[4] = { 1.6,0.15,0.85,1.45 };
    double* carP1wheeloff = AMwheeloff;
    double* carP2wheeloff = Marussiawheeloff;


    carSLsc[0] = CreateVehicle(spaceCARSL, world, AMpos.x, AMpos.y, AMpos.z, AMwheeloff);
    carSLsc[1] = CreateVehicle(spaceCARSL, world, Marussiapos.x, Marussiapos.y, Marussiapos.z, Marussiawheeloff);
    carSLsc[2] = CreateVehicle(spaceCARSL, world, BMWi8pos.x, BMWi8pos.y, BMWi8pos.z, BMWi8wheeloff);

    //FILE* fptr;
    //fptr = fopen("waypoints.txt", "w");

    Vector3 waypoint[44];

    waypoint[0] = (Vector3){ 118.719,-6.218,173.879 };
    waypoint[1] = (Vector3){ 115.051,-6.186,147.493 };
    waypoint[2] = (Vector3){ 111.146,-5.872,128.098 };
    waypoint[3] = (Vector3){ 106.872,-5.083,106.504 };
    waypoint[4] = (Vector3){ 104.613,-3.636,80.435 };
    waypoint[5] = (Vector3){ 103.491,-2.670,61.030 };
    waypoint[6] = (Vector3){ 100.436,-1.904,34.146 };
    waypoint[7] = (Vector3){ 80.396,-1.856,18.974 };
    waypoint[8] = (Vector3){ 55.559,-2.151,6.571 };
    waypoint[9] = (Vector3){ 31.624,-2.122,-20.370 };
    waypoint[10] = (Vector3){ 29.035,-1.810,-45.561 };
    waypoint[11] = (Vector3){ 35.812,0.540,-88.303 };
    waypoint[12] = (Vector3){ 57.248,2.751,-115.669 };
    waypoint[13] = (Vector3){ 75.400,3.899,-129.362 };
    waypoint[14] = (Vector3){ 97.722,5.678,-148.041 };
    waypoint[15] = (Vector3){ 126.368,7.613,-170.657 };
    waypoint[16] = (Vector3){ 121.647,7.573,-187.444 };
    waypoint[17] = (Vector3){ 99.010,6.477,-193.176 };
    waypoint[18] = (Vector3){ 58.772,4.898,-197.537 };
    waypoint[19] = (Vector3){ 34.417,4.108,-193.334 };
    waypoint[20] = (Vector3){ 3.746,3.177,-181.651 };
    waypoint[21] = (Vector3){ -27.108,2.468,-174.133 };
    waypoint[22] = (Vector3){ -50.096,1.852,-169.013 };
    waypoint[23] = (Vector3){ -65.686,0.877,-153.801 };
    waypoint[24] = (Vector3){ -51.318,0.586,-146.536 };
    waypoint[25] = (Vector3){ -26.070,0.782,-134.494 };
    waypoint[26] = (Vector3){ -26.404,-0.099,-101.459 };
    waypoint[27] = (Vector3){ -36.324,-3.042,-62.711 };
    waypoint[28] = (Vector3){ -43.081,-5.498,-21.555 };
    waypoint[29] = (Vector3){ -47.949,-5.682,6.331 };
    waypoint[30] = (Vector3){ -54.335,-6.150,34.994 };
    waypoint[31] = (Vector3){ -59.149,-6.777,62.021 };
    waypoint[32] = (Vector3){ -62.647,-7.575,94.978 };
    waypoint[33] = (Vector3){ -60.971,-8.446,122.479 };
    waypoint[34] = (Vector3){ -52.048,-9.047,148.822 };
    waypoint[35] = (Vector3){ -32.233,-9.072,168.424 };
    waypoint[36] = (Vector3){ -14.878,-9.290,185.018 };
    waypoint[37] = (Vector3){ 6.265,-9.513,200.361 };
    waypoint[38] = (Vector3){ 31.164,-9.459,202.823 };
    waypoint[39] = (Vector3){ 49.551,-9.179,193.120 };
    waypoint[40] = (Vector3){ 64.950,-8.514,173.876 };
    waypoint[41] = (Vector3){ 76.390,-7.420,152.000 };
    waypoint[42] = (Vector3){ 86.826,-5.937,120.567 };
    waypoint[43] = (Vector3){ 94.833,-4.652,97.582 };

    FILE* chptr;
    chptr = fopen("checkpoints.txt", "r");
    Vector3 checkpoints[22];
    for (int i = 0; i < 22; i++) {
        float x, y, z;
        char ch[50];
        fgets(ch, 50, chptr);
        sscanf(ch, "%f %f %f", &x, &y, &z);
        checkpoints[i] = (Vector3){ x,y,z };
    }

    Vector3 waypoints[174];
    FILE* fwpts;
    fwpts = fopen("waypoints.txt", "r");
    for (int i = 0; i < 174; i++) {
        float x, y, z;
        char s[50];
        fgets(s, 50, fwpts);
        sscanf(s, "%f %f %f", &x, &y, &z);
        waypoints[i] = (Vector3){ x,y,z };
    }
                              
    
    
    while (!WindowShouldClose())                
    {
        
        

        //if (IsKeyPressed(KEY_SPACE)) PlaySound(fxWav);      
        //if (IsKeyPressed(KEY_ENTER)) PlaySound(fxOgg);
        switch (currentScreen) {
            case SPLASH_RL: {
                SetTargetFPS(12);
                UpdateMusicStream(splashRM);
                if (!IsMusicStreamPlaying(splashRM)) {
                    PlayMusicStream(splashRM);
                }
                if (nSplhR == 50) {
                    currentScreen = SPLASH_ODE;
                    StopMusicStream(splashRM);
                }
                
                BeginDrawing();
                ClearBackground(RAYWHITE);
                DrawTextureEx(SplashRaytex[nSplhR], (Vector2) {0.0f,0.0f}, 0, 2.0f, WHITE);
                EndDrawing();
                nSplhR++;
            }break;
            case SPLASH_ODE: {
                SetTargetFPS(12);
                UpdateMusicStream(splashGTM);
                if (!IsMusicStreamPlaying(splashGTM)) {
                    PlayMusicStream(splashGTM);
                }
                if (nSplGT == 70) {
                    currentScreen = MENU;
                    StopMusicStream(splashGTM);
                }
                
                BeginDrawing();
                ClearBackground(RAYWHITE);
                DrawTextureEx(SplashGTtex[nSplGT], (Vector2) { 0.0f, 0.0f }, 0, 2.0f, WHITE);
                EndDrawing();
                nSplGT++;
            }break;
            case MENU:
            {
                SetTargetFPS(60);
                UpdateMusicStream(music);
                if (!IsMusicStreamPlaying(music)) {
                    PlayMusicStream(music);
                    mainmenuBGM++;
                }
                mousePoint = GetMousePosition();
                freerideBtnAction = 0;
                if (IsKeyDown(KEY_F)) {
                    ToggleFullscreen();
                }
                BeginDrawing();
                ClearBackground(RAYWHITE);
                if (nBG == 23) {
                    nBG = 0;
                }
                if (nmT == 4) {
                    nmT = 0;;
                }
                DrawTexture(menutexBG[nBG], 0, 0, WHITE);
                DrawTexture(menutexT[nmT], screenWidth/15-40, screenHeight/3.6 - 150, WHITE);
                if (menuRM2Pback == 0) {
                    DrawRectangleRounded((Rectangle) { screenWidth / 15, screenHeight / 3.6, 400, 50 }, 0.6f, 0, Fade(BLUE, 0.8));
                }
                else {
                    DrawRectangleRounded((Rectangle) { screenWidth / 15, screenHeight / 3.6, 580, 50 }, 0.6f, 0, Fade(BLUE, 0.8));
                }
                if (menuRMAIback == 0) {
                    DrawRectangleRounded((Rectangle) { screenWidth / 15, screenHeight / 3.6+80, 400, 50 }, 0.6f, 0, Fade(BLUE, 0.8));
                }
                else {
                    DrawRectangleRounded((Rectangle) { screenWidth / 15, screenHeight / 3.6+80, 580, 50 }, 0.6f, 0, Fade(BLUE, 0.8));
                }
                if (menuFRback == 0) {
                    DrawRectangleRounded((Rectangle) { screenWidth / 15, screenHeight / 3.6 +160, 400, 50 }, 0.6f, 0, Fade(BLUE, 0.8));
                }
                else {
                    DrawRectangleRounded((Rectangle) { screenWidth / 15, screenHeight / 3.6 + 160, 500, 50 }, 0.6f, 0, Fade(BLUE, 0.8));
                }
                //DrawRectangleRounded((Rectangle) { screenWidth/15, screenHeight/3.6+240, 400, 50 }, 0.6f, 0, Fade(BLUE, 0.8));
                //DrawRectangle(screenWidth/15, screenHeight/3.6, 400, 50, Fade(BLUE, 0.8));
                //DrawRectangle(screenWidth / 15, screenHeight / 3.6+80, 400, 50, Fade(BLUE, 0.8));
                //DrawRectangle(screenWidth / 15, screenHeight / 3.6+160, 400, 50, Fade(BLUE,
                if (CheckCollisionPointRec(mousePoint, (Rectangle) { screenWidth / 15, screenHeight / 3.6 , 400, 50 })) {
                    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
                        if (menuRM2Pback == 0) {
                            DrawRectangleRoundedLines((Rectangle) { screenWidth / 15, screenHeight / 3.6, 400, 50 }, 0.6f, 0, 3.0f, Fade(DARKGREEN, 0.8));
                        }
                        else {
                            DrawRectangleRoundedLines((Rectangle) { screenWidth / 15, screenHeight / 3.6, 580, 50 }, 0.6f, 0, 3.0f, Fade(DARKGREEN, 0.8));
                        }
                    }
                    else {
                        if (menuRM2Pback == 0) {
                            DrawRectangleRoundedLines((Rectangle) { screenWidth / 15, screenHeight / 3.6, 400, 50 }, 0.6f, 0, 3.0f, Fade(DARKBLUE, 0.8));
                        }
                        else {
                            DrawRectangleRoundedLines((Rectangle) { screenWidth / 15, screenHeight / 3.6, 580, 50 }, 0.6f, 0, 3.0f, Fade(DARKBLUE, 0.8));
                        }
                    }
                    if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {                      
                        currentScreen = CARSL;
                        nextScreen = RACEMODESPLIT;
                        PauseMusicStream(music);
                    }
                }
                else {
                    
                }
                if (CheckCollisionPointRec(mousePoint, (Rectangle) { screenWidth / 15, screenHeight / 3.6+80, 400, 50 })) {
                    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
                        if (menuRMAIback == 0) {
                            DrawRectangleRoundedLines((Rectangle) { screenWidth / 15, screenHeight / 3.6 + 80, 400, 50 }, 0.6f, 0, 3.0f, Fade(DARKGREEN, 0.8));
                        }
                        else {
                            DrawRectangleRoundedLines((Rectangle) { screenWidth / 15, screenHeight / 3.6 + 80, 580, 50 }, 0.6f, 0, 3.0f, Fade(DARKGREEN, 0.8));
                        }
                    }
                    else {
                        if (menuRMAIback == 0) {
                            DrawRectangleRoundedLines((Rectangle) { screenWidth / 15, screenHeight / 3.6 + 80, 400, 50 }, 0.6f, 0, 3.0f, Fade(DARKBLUE, 0.8));
                        }
                        else {
                            DrawRectangleRoundedLines((Rectangle) { screenWidth / 15, screenHeight / 3.6 + 80, 580, 50 }, 0.6f, 0, 3.0f, Fade(DARKBLUE, 0.8));
                        }
                    }
                    if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
                        currentScreen = RACEMODEAI;
                        PauseMusicStream(music);
                    }
                }
                else {

                }
                if (CheckCollisionPointRec(mousePoint, (Rectangle) { screenWidth / 15, screenHeight / 3.6 + 160, 400, 50 })) {
                    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
                        freerideBtnState = 2;
                        if (menuFRback == 0) {
                            DrawRectangleRoundedLines((Rectangle) { screenWidth / 15, screenHeight / 3.6 + 160, 400, 50 }, 0.6f, 0, 3.0f, Fade(DARKGREEN, 0.8));
                        }
                        else {
                            DrawRectangleRoundedLines((Rectangle) { screenWidth / 15, screenHeight / 3.6 + 160, 500, 50 }, 0.6f, 0, 3.0f, Fade(DARKGREEN, 0.8));
                        }
                    }
                    else {
                        freerideBtnState = 1;
                        if (menuFRback == 0) {
                            DrawRectangleRoundedLines((Rectangle) { screenWidth / 15, screenHeight / 3.6 + 160, 400, 50 }, 0.6f, 0, 3.0f, Fade(DARKBLUE, 0.8));
                        }
                        else {
                            DrawRectangleRoundedLines((Rectangle) { screenWidth / 15, screenHeight / 3.6 + 160, 500, 50 }, 0.6f, 0, 3.0f, Fade(DARKBLUE, 0.8));
                        }
                    }
                    if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
                        freerideBtnAction = 1;
                        currentScreen = FREERIDE;
                        PauseMusicStream(music);
                    }
                }
                else {
                    freerideBtnState = 0;
                }
                if (menuRM2Pback == 0) {
                    DrawText("Race Mode 2P", screenWidth / 15 + 50, screenHeight / 3.6 + 5, 40, MAROON);
                }
                else {
                    DrawText("Resume Race Mode 2P", screenWidth / 15 + 50, screenHeight / 3.6 + 5, 40, MAROON);
                }
                if (menuRMAIback == 0) {
                    DrawText("Race Mode AI", screenWidth / 15 + 50, screenHeight / 3.6 + 5+80, 40, MAROON);
                }
                else {
                    DrawText("Resume Race Mode AI", screenWidth / 15 + 50, screenHeight / 3.6 + 5+80, 40, MAROON);
                }
                if (menuFRback == 0) {
                    DrawText("Free Ride", screenWidth / 15 + 50, screenHeight / 3.6 + 160 + 5, 40, MAROON);
                }
                else {
                    DrawText("Resume Free Ride", screenWidth / 15 + 50, screenHeight / 3.6 + 160 + 5, 40, MAROON);
                }

                //DrawText("Options", screenWidth / 15+50, screenHeight / 3.6+240+5, 40, MAROON);
                nBG++;
                nmT++;
                EndDrawing();
            }break;
                case STORYMODE: {

                }break;
                case CARSL: {
                    for (int i = 0; i < 3; i++) {
                        for (int u = 0; u < 5; u++) {
                            dBodySetLinearVel(carSLsc[i]->bodies[u], 0, 0, 0);
                            dBodySetAngularVel(carSLsc[i]->bodies[u], 0, 0, 0);                          
                        }
                        const dReal* qP1 = dBodyGetQuaternion(carSLsc[i]->bodies[0]);
                        double z0P1 = 2.0f * (qP1[0] * qP1[3] + qP1[1] * qP1[2]);
                        double z1P1 = 1.0f - 2.0f * (qP1[1] * qP1[1] + qP1[3] * qP1[3]);
                        double rollP1 = atan2f(z0P1, z1P1);
                        if (fabs(rollP1) > (M_PI_2 - 0.001)) {
                            unflipVehicle(carSLsc[i]);
                        }
                        dReal* cp = dBodyGetPosition(carSLsc[i]->bodies[0]);
                        if (i == 0) {
                            if (cp[0] != AMpos.x && cp[1] != AMpos.y && cp[2] != AMpos.z) {
                                dBodySetPosition(carSLsc[i]->bodies[0], AMpos.x, AMpos.y+0.3, AMpos.z);
                            }
                        }
                        else if (i == 1) {
                            if (cp[0] != Marussiapos.x && cp[1] != Marussiapos.y && cp[2] != Marussiapos.z) {
                                dBodySetPosition(carSLsc[i]->bodies[0], Marussiapos.x, Marussiapos.y+0.3, Marussiapos.z);
                            }
                        }
                        else if (i == 2) {
                            if (cp[0] != BMWi8pos.x && cp[1] != BMWi8pos.y && cp[2] != BMWi8pos.z) {
                                dBodySetPosition(carSLsc[i]->bodies[0], BMWi8pos.x, BMWi8pos.y+0.3, BMWi8pos.z);
                            }
                        }
                    }
                    
                    
                    if (IsKeyReleased(KEY_F)) {
                        ToggleFullscreen();
                    }

                    
                    if (IsCRSLAMCarCreated == 0) {       
                        cameraCARSLSPLIT.target = AMpos;
                    }
                    mouseDelta = GetMouseDelta();
                    mousePoint = GetMousePosition();
                    yaw += mouseDelta.x * sensitivity;
                    pitchcar += mouseDelta.y * sensitivity;
                    if (pitchcar > 89.0f) {
                        pitchcar = 89.0f;
                    }
                    if (pitchcar < -0.8f) {
                        pitchcar = -0.8f;
                    }

                    mousePos.x = cosf(DEG2RAD * yaw) * cosf(DEG2RAD * pitchcar);
                    mousePos.y = sin(DEG2RAD * pitchcar);
                    mousePos.z = sinf(DEG2RAD * yaw) * cosf(DEG2RAD * pitchcar);
                    if (CheckCollisionPointRec(mousePoint, (Rectangle) { screenWidth - 120, screenHeight - 80, 120, 40 })) {
                        if (nextScreen == RACEMODESPLIT) {
                            if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT) && IsP1CarSL == 0) {
                                IsP1CarSL++;
                            }
                            else if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT) && IsP1CarSL == 1) {
                                currentScreen = nextScreen;
                            }
                        }
                    }
                    if ((IsKeyReleased(KEY_RIGHT)&&currentCar.x==AMpos.x)||(IsKeyReleased(KEY_LEFT)&&currentCar.x==BMWi8pos.x)) {
                        currentCar= Marussiapos;
                        if (IsP1CarSL == 0&&nextScreen==RACEMODESPLIT) {
                            carMPlayer1 = MARUSSIA;
                            carP1wheeloff = Marussiawheeloff;
                        }      
                        else if (IsP1CarSL == 1 && nextScreen == RACEMODESPLIT) {
                            carMPlayer2 = MARUSSIA;
                            carP2wheeloff = Marussiawheeloff;
                        }
                    }
                    else if (IsKeyReleased(KEY_LEFT)&&currentCar.x==Marussiapos.x) {
                        currentCar = AMpos;
                        if (IsP1CarSL == 0 && nextScreen == RACEMODESPLIT) {
                            carMPlayer1 = Amuscle;
                            carP1wheeloff = AMwheeloff;
                        }
                        else if (IsP1CarSL == 1 && nextScreen == RACEMODESPLIT) {
                            carMPlayer2 = Amuscle;
                            carP2wheeloff =AMwheeloff;
                        }
                    }
                    else if (IsKeyReleased(KEY_RIGHT) && currentCar.x == Marussiapos.x) {
                        currentCar = BMWi8pos;
                        if (IsP1CarSL == 0 && nextScreen == RACEMODESPLIT) {
                            carMPlayer1 = BMW;
                            carP1wheeloff = BMWi8wheeloff;
                        }
                        else if (IsP1CarSL == 1 && nextScreen == RACEMODESPLIT) {
                            carMPlayer2 = BMW;
                            carP2wheeloff = BMWi8wheeloff;
                        }
                    }
                    
                    cameraCARSLSPLIT.target = currentCar;
                    
                    cameraCARSLSPLIT.position.x -= (cameraCARSLSPLIT.position.x-(cameraCARSLSPLIT.target.x + 8.0 * mousePos.x))*0.1;
                    cameraCARSLSPLIT.position.y -= (cameraCARSLSPLIT.position.y  - (cameraCARSLSPLIT.target.y + 8.0 * mousePos.y)) * 0.1;
                    cameraCARSLSPLIT.position.z -= (cameraCARSLSPLIT.position.z - (cameraCARSLSPLIT.target.z + 8.0 * mousePos.z)) * 0.1;
                    UpdateCamera(&cameraCARSLSPLIT);
                    SetShaderValue(shader, shader.locs[SHADER_LOC_VECTOR_VIEW], &cameraCARSLSPLIT.position.x, SHADER_UNIFORM_VEC3);

                    frameTime += GetFrameTime();
                    int pSteps = 0;
                    physTime = GetTime();

                    while (frameTime > physSlice) {                        
                        dSpaceCollide(spaceCARSL, 0, &nearCallback);
                        dWorldQuickStep(world, physSlice);  // NB fixed time step is important
                        dJointGroupEmpty(contactgroup);

                        frameTime -= physSlice;
                        pSteps++;
                        if (pSteps > maxPsteps) {
                            frameTime = 0;
                            break;
                        }
                    }

                    physTime = GetTime() - physTime;

                    BeginDrawing();
                    ClearBackground(RAYWHITE);
                    BeginBlendMode(BLEND_ALPHA);
                    BeginMode3D(cameraCARSLSPLIT);
                    rlDisableBackfaceCulling();
                    rlDisableDepthMask();
                    DrawModel(skybox, (Vector3) { 0, 0, 0 }, 1.0f, WHITE);
                    rlEnableBackfaceCulling();
                    rlEnableDepthMask();
                    DrawModel(ground, (Vector3) { 0.0f, 0.0f, 0.0f }, 1.0f, WHITE);
                    drawAllSpaceCarSLGeoms(spaceCARSL, carSLsc);
                    //DrawCarModelSL(AM, WheelAMDCL, WheelAMDCR, AMpos, AMwheeloffSL);
                    //DrawCarModelSL(Marussia, WheelMarussiaL, WheelMarussiaR, Marussiapos, MarussiawheeloffSL);
                    //DrawCarModelSL(BMWi8, WheelBMWi8L, WheelBMWi8R, BMWi8pos, BMWi8wheeloffSL);
                    EndMode3D();
                    EndBlendMode();
                    if (pSteps > maxPsteps) DrawText("WARNING CPU overloaded lagging real time", 10, 0, 20, RED);
                    DrawText(TextFormat("%2i FPS", GetFPS()), 10, 20, 20, BLACK);
                    DrawText("Select Car", screenWidth / 12, screenHeight / 12, 55, MAROON);
                    DrawRectangleRounded((Rectangle) { screenWidth - 120, screenHeight - 80, 120, 40 }, 0.75, 0, GREEN);
                    if (IsP1CarSL == 0&&nextScreen==RACEMODESPLIT) {
                        DrawText("Select", screenWidth - 120 + 15, screenHeight - 80 + 10, 30, VIOLET);
                    }
                    else if (IsP1CarSL == 1&&nextScreen==RACEMODESPLIT) {
                        DrawText("Select", screenWidth - 120 + 15, screenHeight - 80 + 10, 30, VIOLET);
                    }
                    if (nextScreen == RACEMODESPLIT&&IsP1CarSL==0) {
                        DrawText("Player 1", screenWidth / 12, screenHeight / 12+60, 50, MAROON);
                    }
                    else if (nextScreen == RACEMODESPLIT && IsP1CarSL == 1) {
                        DrawText("Player 2", screenWidth / 12, screenHeight / 12 + 60, 50, MAROON);
                    }
                    if (currentCar.x == AMpos.x) {
                        DrawText("American Muscle", screenWidth / 3.2, screenHeight-100, 45, RED);
                    }
                    else if (currentCar.x == Marussiapos.x) {
                        DrawText("Marussia", screenWidth / 3.2, screenHeight-100 , 45, RED);
                    }
                    else if (currentCar.x == BMWi8pos.x) {
                        DrawText("BMW i8", screenWidth / 3.2, screenHeight - 100, 45, RED);
                    }
                    EndDrawing();
                }break;
                case RACEMODESPLIT:{
                    
                    if (IsP1CarCreated == 0) {
                        carPlayer1 = CreateVehicle(space2P, world, 130, -5.5, 200,carP1wheeloff);
                        IsP1CarCreated++;
                        
                    }
                    if (IsP2CarCreated == 0) {
                        carPlayer2 = CreateVehicle(space2P, world, 124.2, -5.9, 206,carP2wheeloff);
                        IsP2CarCreated++;
                    }
                    
                    mousePoint = GetMousePosition();

                    
                    const dReal* qP1 = dBodyGetQuaternion(carPlayer1->bodies[0]);
                    double z0P1 = 2.0f * (qP1[0] * qP1[3] + qP1[1] * qP1[2]);
                    double z1P1 = 1.0f - 2.0f * (qP1[1] * qP1[1] + qP1[3] * qP1[3]);
                    double rollP1 = atan2f(z0P1, z1P1);
                    if (fabs(rollP1) > (M_PI_2 - 0.001)) {
                        carP1Flipped++;
                    }
                    else {
                        carP1Flipped = 0;
                    }

                    if (carP1Flipped > 100) {
                        unflipVehicle(carPlayer1);
                    }
                    const dReal* qP2 = dBodyGetQuaternion(carPlayer2->bodies[0]);
                    double z0P2 = 2.0f * (qP2[0] * qP2[3] + qP2[1] * qP2[2]);
                    double z1P2 = 1.0f - 2.0f * (qP2[1] * qP2[1] + qP2[3] * qP2[3]);
                    double rollP2 = atan2f(z0P2, z1P2);
                    if (fabs(rollP2) > (M_PI_2 - 0.001)) {
                        carP2Flipped++;
                    }
                    else {
                        carP2Flipped = 0;
                    }

                    if (carP2Flipped > 100) {
                        unflipVehicle(carPlayer2);
                    }

                    accelP1 *= .99;
                    if (IsKeyDown(KEY_UP)) accelP1 += 2;
                    if (IsKeyDown(KEY_DOWN)) accelP1 -= 2;
                    if (accelP1 > 70) accelP1 = 70;
                    if (accelP1 < -15) accelP1 = -15;

                    if (IsKeyDown(KEY_F)) {
                        ToggleFullscreen();
                    }

                    if (IsKeyDown(KEY_RIGHT)) steerP1 -= .005;
                    if (IsKeyDown(KEY_LEFT)) steerP1 += .005;
                    if (!IsKeyDown(KEY_RIGHT) && !IsKeyDown(KEY_LEFT)) steerP1 *= .5;
                    if (steerP1 > .5) steerP1 = .5;
                    if (steerP1 < -.5) steerP1 = -.5;

                    accelP2 *= .99;
                    if (IsKeyDown(KEY_W)) accelP2 += 2;
                    if (IsKeyDown(KEY_S)) accelP2 -= 2;
                    if (accelP2 > 70) accelP2 = 70;
                    if (accelP2 < -15) accelP2 = -15;

                   

                    if (IsKeyDown(KEY_D)) steerP2 -= .005;
                    if (IsKeyDown(KEY_A)) steerP2 += .005;
                    if (!IsKeyDown(KEY_D) && !IsKeyDown(KEY_A)) steerP2 *= .5;
                    if (steerP2 > .5) steerP2 = .5;
                    if (steerP2 < -.5) steerP2 = -.5;

                    if (CheckCollisionPointRec(mousePoint, (Rectangle) { 10, 10, 80, 40 })) {
                        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
                            currentScreen = MENU;
                            menuRM2Pback = 1;
                        }
                    }
                    else {
                        menuRM2Pback = 1;
                    }

                    if (carwin2P == 0 || carwin2P == 1) {
                        if (CheckCollisionPointRec(mousePoint, (Rectangle) { screenWidth / 2 - 230, screenHeight / 2 + 80, 250, 50 })) {
                            if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
                                currentScreen = MENU;
                                menuRM2Pback = 0;
                                for (int i = 0; i < 6; i++) {
                                    dBodyDestroy(carPlayer1->bodies[i]);
                                    dBodyDestroy(carPlayer2->bodies[i]);
                                }
                                RL_FREE(carPlayer1);
                                RL_FREE(carPlayer2);
                                IsP1CarCreated = 0;
                                IsP2CarCreated = 0;
                                carwin2P = -1;
                                IsP1CarSL = 0;
                                carP1chptIndex = 0;
                                carP2chptIndex = 0;
                            }
                        }
                        if (CheckCollisionPointRec(mousePoint, (Rectangle) { screenWidth / 2 + 120, screenHeight / 2 + 80, 210, 50 })) {
                            if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
                                for (int i = 0; i < 6; i++) {
                                    dBodyDestroy(carPlayer1->bodies[i]);
                                    dBodyDestroy(carPlayer2->bodies[i]);
                                    dGeomDestroy(carPlayer1->geoms[i]);
                                    dGeomDestroy(carPlayer2->geoms[i]);
                                }
                                RL_FREE(carPlayer1);
                                RL_FREE(carPlayer2);
                                IsP1CarCreated = 0;
                                IsP2CarCreated = 0;
                                carwin2P = -1;
                                IsP1CarSL = 0;
                                carP1chptIndex = 0;
                                carP2chptIndex = 0;
                            }
                        }
                    }

                    const double* vP1 = dBodyGetLinearVel(carPlayer1->bodies[0]);
                    float velP1 = Vector3Length((Vector3) { vP1[0], vP1[1], vP1[2] }) * 2.23693629f;
                    updateVehicle(carPlayer1, accelP1, 800.0, steerP1, 10.0);
                    speedoac1 = 40 - ((240.0) / (75)) * (accelP1 - 75);
                    speedov1 = 40 - ((240.0) / (85)) * (velP1 - 85);

                    const dReal* cpP1 = dBodyGetPosition(carPlayer1->bodies[0]);
                    cameraPlayer1.target = (Vector3){ cpP1[0],cpP1[1] + 1,cpP1[2] };
                    if (carP1chptIndex == 21) {
                        carwin2P = 0;
                    }
                    carP1chptIndex = CheckPointIndex(cpP1, checkpoints, carP1chptIndex);
                    float lerp = 0.1f;

                    dVector3 coP1;
                    dBodyGetRelPointPos(carPlayer1->bodies[0], -8, 3, 0, coP1);
                    dReal* rotmatP1 = dBodyGetRotation(carPlayer1->bodies[0]);

                    const double* vP2 = dBodyGetLinearVel(carPlayer2->bodies[0]);
                    float velP2 = Vector3Length((Vector3) { vP2[0], vP2[1], vP2[2] }) * 2.23693629f;
                    updateVehicle(carPlayer2, accelP2, 800.0, steerP2, 10.0);
                    speedoac2 = 40 - ((240.0) / (75)) * (accelP2 - 75);
                    speedov2 = 40 - ((240.0) / (85)) * (velP2 - 85);

                    const dReal* cpP2 = dBodyGetPosition(carPlayer2->bodies[0]);
                    cameraPlayer2.target = (Vector3){ cpP2[0],cpP2[1] + 1,cpP2[2] };
                    if (carP2chptIndex == 21) {
                        carwin2P = 1;
                    }
                    carP2chptIndex = CheckPointIndex(cpP2, checkpoints, carP2chptIndex);

                    

                    dVector3 coP2;
                    dBodyGetRelPointPos(carPlayer2->bodies[0], -8, 3, 0, coP2);
                    dReal* rotmatP2 = dBodyGetRotation(carPlayer2->bodies[0]);

                    /*printf("Line 1 : %.2f %.2f %.2f %.2f \n", rotmat[0], rotmat[1], rotmat[2], rotmat[3]);
                    printf("Line 2 : %.2f %.2f %.2f %.2f \n", rotmat[4], rotmat[5], rotmat[6], rotmat[7]);
                    printf("Line 3 : %.2f %.2f %.2f %.2f \n", rotmat[8], rotmat[9], rotmat[10], rotmat[11]);
                    printf("Line 4 : %.2f %.2f %.2f %.2f \n", rotmat[12], rotmat[13], rotmat[14], rotmat[15]);*/




                    //printf("%.3f %.3f %.3f \n", cp[0], cp[1], cp[2]);


                    mouseDelta = GetMouseDelta();
                    yaw += mouseDelta.x * sensitivity;
                    pitchcar += mouseDelta.y * sensitivity;
                    /*if (IsKeyDown(KEY_A)) {
                        yaw -= 0.8;
                    }
                    else if (IsKeyDown(KEY_D)) {
                        yaw += 0.8;
                    }*/








                    if (pitchcar > 89.0f) {
                        pitchcar = 89.0f;
                    }
                    if (pitchcar < -5.0f) {
                        pitchcar = -5.0f;
                    }

                    mousePos.x = cosf(DEG2RAD * yaw) * cosf(DEG2RAD * pitchcar);
                    mousePos.y = sin(DEG2RAD * pitchcar);
                    mousePos.z = sinf(DEG2RAD * yaw) * cosf(DEG2RAD * pitchcar);


                    /*lights[2].target.x = co[0];
                    lights[2].target.y = co[1];
                    lights[2].target.z = co[2];
                    lights[2].position.x = lightposL[0];
                    lights[2].position.y = lightposL[1];
                    lights[2].position.z = lightposL[2];


                    UpdateLightValues(shader, lights[2]);*/
                    if (accelP1 > 0 && (mouseDelta.x == 0) && (mouseDelta.y == 0)) {
                        cameraPlayer1.position.x -= (cameraPlayer1.position.x - coP1[0]) * lerp + 0.0 * mousePos.x;// * (1/ft);
                        cameraPlayer1.position.y -= (cameraPlayer1.position.y - coP1[1]) * lerp + 0.0 * mousePos.y;// * (1/ft);
                        cameraPlayer1.position.z -= (cameraPlayer1.position.z - coP1[2]) * lerp + 0.0 * mousePos.z;// * (1/ft);
                    }
                    else {
                        cameraPlayer1.position.x -= (cameraPlayer1.position.x - (cameraPlayer1.target.x + 8.0 * mousePos.x)) * lerp;
                        cameraPlayer1.position.y -= (cameraPlayer1.position.y - (cameraPlayer1.target.y + 8.0 * mousePos.y)) * lerp;
                        cameraPlayer1.position.z -= (cameraPlayer1.position.z - (cameraPlayer1.target.z + 8.0 * mousePos.z)) * lerp;
                    }

                    if (accelP2 > 0 && (mouseDelta.x == 0) && (mouseDelta.y == 0)) {
                        cameraPlayer2.position.x -= (cameraPlayer2.position.x - coP2[0]) * lerp + 0.0 * mousePos.x;// * (1/ft);
                        cameraPlayer2.position.y -= (cameraPlayer2.position.y - coP2[1]) * lerp + 0.0 * mousePos.y;// * (1/ft);
                        cameraPlayer2.position.z -= (cameraPlayer2.position.z - coP2[2]) * lerp + 0.0 * mousePos.z;// * (1/ft);
                    }
                    else {
                        cameraPlayer2.position.x -= (cameraPlayer2.position.x - (cameraPlayer2.target.x + 8.0 * mousePos.x)) * lerp;
                        cameraPlayer2.position.y -= (cameraPlayer2.position.y - (cameraPlayer2.target.y + 8.0 * mousePos.y)) * lerp;
                        cameraPlayer2.position.z -= (cameraPlayer2.position.z - (cameraPlayer2.target.z + 8.0 * mousePos.z)) * lerp;
                    }


                   

                    mousePos.x = mousePos.x * 0.8;
                    mousePos.y = mousePos.y * 0.8;
                    mousePos.z = mousePos.z * 0.8;


                    bool spcdn = IsKeyDown(KEY_SPACE);

                    for (int i = 0; i < numObj; i++) {
                        const dReal* pos = dBodyGetPosition(obj[i]);
                        if (spcdn) {
                            // apply force if the space key is held
                            const dReal* v = dBodyGetLinearVel(obj[0]);
                            if (v[1] < 10 && pos[1] < 10) { // cap upwards velocity and don't let it get too high
                                dBodyEnable(obj[i]); // case its gone to sleep
                                dMass mass;
                                dBodyGetMass(obj[i], &mass);
                                // give some object more force than others
                                float f = (6 + (((float)i / numObj) * 4)) * mass.mass;
                                dBodyAddForce(obj[i], rndf(-f, f), f * 4, rndf(-f, f));
                            }
                        }


                        if (pos[1] < -10) {
                            // teleport back if fallen off the ground
                            dBodySetPosition(obj[i], dRandReal() * 10 - 5,
                                12 + rndf(1, 2), dRandReal() * 10 - 5);
                            dBodySetLinearVel(obj[i], 0, 0, 0);
                            dBodySetAngularVel(obj[i], 0, 0, 0);
                        }
                    }
                    if (cpP1[1] < -30) {
                        dBodySetPosition(carPlayer1->bodies[0], 130, 1, 190);
                        dBodySetLinearVel(carPlayer1->bodies[0], 0, 0, 0);
                        dBodySetAngularVel(carPlayer1->bodies[0], 0, 0, 0);
                    }
                    if (cpP2[1] < -30) {
                        dBodySetPosition(carPlayer2->bodies[0], 130, 1, 190);
                        dBodySetLinearVel(carPlayer2->bodies[0], 0, 0, 0);
                        dBodySetAngularVel(carPlayer2->bodies[0], 0, 0, 0);
                    }



                    UpdateCamera(&cameraPlayer1);    
                    UpdateCamera(&cameraPlayer2);  // Update camera

                    if (IsKeyPressed(KEY_L)) { lights[0].enabled = !lights[0].enabled; UpdateLightValues(shader, lights[0]); }

                    // update the light shader with the camera view position
                    SetShaderValue(shader, shader.locs[SHADER_LOC_VECTOR_VIEW], &cameraPlayer1.position.x, SHADER_UNIFORM_VEC3);
                    SetShaderValue(shader, shader.locs[SHADER_LOC_VECTOR_VIEW], &cameraPlayer2.position.x, SHADER_UNIFORM_VEC3);
                    frameTime += GetFrameTime();
                    int pSteps = 0;
                    physTime = GetTime();

                    while (frameTime > physSlice) {
                        // check for collisions
                        // TODO use 2nd param data to pass custom structure with
                        // world and space ID's to avoid use of globals...
                        dSpaceCollide(space2P, 0, &nearCallback);

                        // step the world
                        dWorldQuickStep(world, physSlice);  // NB fixed time step is important
                        dJointGroupEmpty(contactgroup);

                        frameTime -= physSlice;
                        pSteps++;
                        if (pSteps > maxPsteps) {
                            frameTime = 0;
                            break;
                        }
                    }

                    physTime = GetTime() - physTime;

                    if (IsFrame == 0) {
                        BeginTextureMode(screenPlayer1);
                        BeginDrawing();

                        ClearBackground(RAYWHITE);
                        BeginBlendMode(BLEND_ALPHA);
                        BeginMode3D(cameraPlayer1);
                        //BeginShaderMode(alphadiscard);
                        rlDisableBackfaceCulling();
                        rlDisableDepthMask();
                        DrawModel(skybox, (Vector3) { 0, 0, 0 }, 1.0f, WHITE);
                        rlEnableBackfaceCulling();
                        rlEnableDepthMask();
                        DrawModel(ground, (Vector3) { 0, 0, 0 }, 1, WHITE);

                        drawAllSpaceTWOPGeoms(space2P, carPlayer1,carPlayer2,carMPlayer1,carMPlayer2);
                        //carPlayer->geoms[0],carPlayer1->geoms[3],carPlayer1->geoms[1]
                        EndMode3D();
                        EndBlendMode();

                        if (pSteps > maxPsteps) DrawText("WARNING CPU overloaded lagging real time", 10, 0, 20, RED);
                        DrawText(TextFormat("%2i FPS", GetFPS()), 10, 20, 20, BLACK);
                        /*DrawText(TextFormat("accel %4.4f", accel), 10, 40, 20, BLACK);
                        DrawText(TextFormat("steer %4.4f", steer), 10, 60, 20, BLACK);
                        if (!antiSway) DrawText("Anti sway bars OFF", 10, 80, 20, RED);
                        DrawText(TextFormat("debug %4.4f %4.4f %4.4f", debug.x, debug.y, debug.z), 10, 100, 20, BLACK);
                        DrawText(TextFormat("Phys steps per frame %i", pSteps), 10, 120, 20, BLACK);
                        DrawText(TextFormat("Phys time per frame %i", physTime), 10, 140, 20, BLACK);
                        DrawText(TextFormat("total time per frame %i", frameTime), 10, 160, 20, BLACK);
                        DrawText(TextFormat("objects %i", numObj), 10, 180, 20, BLACK);*/
                        DrawText(TextFormat("Checkpoint:%d", carP1chptIndex+1), 10, screenHeight - 20, 20, BLACK);
                        DrawRing((Vector2) { screenWidth/2.0f - 150, screenHeight - 170 }, 100.0f, 112.0f, 40, 280, 0, Fade(WHITE, 0.6f));
                        DrawRing((Vector2) { screenWidth/2.0f - 150, screenHeight - 170 }, 102.0f, 110.0f, speedoac1, 280, 0, Fade(RED, 0.6f));
                        DrawRing((Vector2) { screenWidth/2.0f - 150, screenHeight - 170 }, 86.0f, 96.0f, 40, 280, 0, Fade(LIGHTGRAY, 0.8f));
                        DrawRing((Vector2) { screenWidth/2.0f - 150, screenHeight - 170 }, 88.0f, 94.0f, speedov1, 280, 0, Fade(DARKBLUE, 0.8f));
                        DrawText(TextFormat("%.0f MPH", velP1), screenWidth/2.0f - 200, screenHeight - 190, 30, VIOLET);

                        //DrawText(TextFormat("roll %.4f", fabs(rollP1)), 10, 200, 20, BLACK);
                        //DrawText(TextFormat("mph %.4f", velP1), 10, 220, 20, BLACK);
                        DrawRectangleRounded((Rectangle) { 10, 10, 80, 40 }, 0.8, 0, Fade(BLUE, 0.8));

                        EndDrawing();
                        EndTextureMode();

                        BeginTextureMode(screenPlayer2);
                        BeginDrawing();

                        ClearBackground(RAYWHITE);
                        BeginBlendMode(BLEND_ALPHA);
                        BeginMode3D(cameraPlayer2);
                        //BeginShaderMode(alphadiscard);
                        rlDisableBackfaceCulling();
                        rlDisableDepthMask();
                        DrawModel(skybox, (Vector3) { 0, 0, 0 }, 1.0f, WHITE);
                        rlEnableBackfaceCulling();
                        rlEnableDepthMask();
                        DrawModel(ground, (Vector3) { 0, 0, 0 }, 1, WHITE);

                        drawAllSpaceTWOPGeoms(space2P, carPlayer2, carPlayer1, carMPlayer2, carMPlayer1);
                        EndMode3D();
                        EndBlendMode();

                        if (pSteps > maxPsteps) DrawText("WARNING CPU overloaded lagging real time", 10, 0, 20, RED);
                        DrawText(TextFormat("Checkpoint:%d", carP2chptIndex+1), 10, screenHeight - 20, 20, BLACK);
                        DrawRing((Vector2) { screenWidth/2.0f - 150, screenHeight - 170 }, 100.0f, 112.0f, 40, 280, 0, Fade(WHITE, 0.6f));
                        DrawRing((Vector2) { screenWidth/2.0f - 150, screenHeight - 170 }, 102.0f, 110.0f, speedoac2, 280, 0, Fade(RED, 0.6f));
                        DrawRing((Vector2) { screenWidth/2.0f - 150, screenHeight - 170 }, 86.0f, 96.0f, 40, 280, 0, Fade(LIGHTGRAY, 0.8f));
                        DrawRing((Vector2) { screenWidth/2.0f - 150, screenHeight - 170 }, 88.0f, 94.0f, speedov2, 280, 0, Fade(DARKBLUE, 0.8f));
                        DrawText(TextFormat("%.0f MPH", velP2), screenWidth/2.0f - 200, screenHeight - 190, 30, VIOLET);

                        //DrawText(TextFormat("roll %.4f", fabs(rollP2)), 10, 200, 20, BLACK);
                        //DrawText(TextFormat("mph %.4f", velP2), 10, 220, 20, BLACK);
                        DrawRectangleRounded((Rectangle) { 10, 10, 80, 40 }, 0.8, 0, Fade(BLUE, 0.8));

                        EndDrawing();
                        EndTextureMode();
                        IsFrame++;
                    }
                    else if (IsFrame == 1) {
                        IsFrame--;
                    }

                    BeginDrawing();
                    ClearBackground(BLACK);
                    if (carwin2P == -1) {
                        DrawTextureRec(screenPlayer2.texture, splitScreenRect, (Vector2) { 0.0f, 0.0f }, WHITE);
                        DrawTextureRec(screenPlayer1.texture, splitScreenRect, (Vector2) { screenWidth / 2.0f, 0.0f }, WHITE);
                    }
                    else if (carwin2P == 0) {
                        ClearBackground(Fade(RAYWHITE,0.85));
                        DrawText("Player 1 Won The Race", screenWidth / 2 - 250, screenHeight / 2-60, 50, BLUE);
                        DrawRectangleRounded((Rectangle) { screenWidth / 2 - 230, screenHeight / 2 + 80, 250, 50 }, 0.7, 0, Fade(RED, 0.85));
                        DrawRectangleRounded((Rectangle) { screenWidth / 2 +120, screenHeight / 2 + 80, 210, 50 }, 0.7, 0, Fade(GREEN, 0.85));
                        DrawText("Back to Menu", screenWidth / 2 - 230+15, screenHeight / 2 + 80+10, 30, ORANGE);
                        DrawText("Ride Again", screenWidth / 2 +120+15, screenHeight / 2 + 80+10, 30, MAGENTA);
                    }
                    else if (carwin2P == 1) {
                        ClearBackground(Fade(RAYWHITE,0.85));
                        DrawText("Player 2 Won The Race", screenWidth / 2 - 250, screenHeight / 2-60, 50, BLUE);
                        DrawRectangleRounded((Rectangle) { screenWidth / 2 - 230, screenHeight / 2 + 80, 250, 50 }, 0.7, 0, Fade(RED, 0.85));
                        DrawRectangleRounded((Rectangle) { screenWidth / 2 + 120, screenHeight / 2 + 80, 210, 50 }, 0.7, 0, Fade(GREEN, 0.85));
                        DrawText("Back to Menu", screenWidth / 2 - 230+15, screenHeight / 2 + 80+10, 30, ORANGE);
                        DrawText("Ride Again", screenWidth / 2 + 120+15, screenHeight / 2 + 80+10, 30, MAGENTA);
                    }
                    EndDrawing();


                }break;
                case RACEMODEAI:
                {
                    if (IsAICarCreated == 0) {
                        AIcar = CreateVehicle(spaceAI, world, 124.2, -5.9, 206,BMWi8wheeloff);
                        IsAICarCreated++;
                    }
                    if (IsRMCarCreatedAIM == 0) {
                        carAIM= CreateVehicle(spaceAI, world, 130, -5.5, 200,Marussiawheeloff);
                        IsRMCarCreatedAIM++;
                    }

                    mousePoint = GetMousePosition();

                    SetShaderValue(l_shader, l_shader.locs[SHADER_LOC_VECTOR_VIEW], (float[3]) { camera.position.x, camera.position.y, camera.position.z }, RL_SHADER_UNIFORM_VEC3);

                    const dReal* q = dBodyGetQuaternion(carAIM->bodies[0]);
                    double z0 = 2.0f * (q[0] * q[3] + q[1] * q[2]);
                    double z1 = 1.0f - 2.0f * (q[1] * q[1] + q[3] * q[3]);
                    double roll = atan2f(z0, z1);
                    if (fabs(roll) > (M_PI_2 - 0.001)) {
                        carFlipped++;
                    }
                    else {
                        carFlipped = 0;
                    }

                    if (carFlipped > 100) {
                        unflipVehicle(carAIM);
                    }

                    const dReal* qAI = dBodyGetQuaternion(carAIM->bodies[0]);
                    double z0AI = 2.0f * (qAI[0] * qAI[3] + qAI[1] * qAI[2]);
                    double z1AI = 1.0f - 2.0f * (qAI[1] * qAI[1] + qAI[3] * qAI[3]);
                    double rollAI = atan2f(z0AI, z1AI);
                    if (fabs(rollAI) > (M_PI_2 - 0.001)) {
                        carFlippedAI++;
                    }
                    else {
                        carFlippedAI = 0;
                    }

                    if (carFlippedAI > 100) {
                        unflipVehicle(AIcar);
                    }


                    accel *= .99;
                    if (IsKeyDown(KEY_UP)) accel += 2;
                    if (IsKeyDown(KEY_DOWN)) accel -= 2;
                    if (accel > 70) accel = 70;
                    if (accel < -15) accel = -15;



                    if (IsKeyDown(KEY_F)) {
                        ToggleFullscreen();
                    }

                    if (IsKeyDown(KEY_RIGHT)) steer -= .005;
                    if (IsKeyDown(KEY_LEFT)) steer += .005;
                    if (!IsKeyDown(KEY_RIGHT) && !IsKeyDown(KEY_LEFT)) steer *= .5;
                    if (steer > .5) steer = .5;
                    if (steer < -.5) steer = -.5;

                    //AI Controls

                    const dReal* AIcp = dBodyGetPosition(AIcar->bodies[0]);
                    const dVector3 AIco;
                    const dVector3 AIside;
                    dBodyGetRelPointPos(AIcar->bodies[0], -5, 0, 0, AIco);
                    dBodyGetRelPointPos(AIcar->bodies[0], 0, 0, -4, AIside);
                    AIAccelFunc = IsAIAccel((Vector3) { AIcp[0], AIcp[1], AIcp[2] }, (Vector3) { AIcp[0] - AIco[0], AIcp[1] - AIco[1], AIcp[2] - AIco[2] },waypoints);
                    AISteerFunc = AISteer((Vector3) { AIcp[0], AIcp[1], AIcp[2] }, (Vector3) { AIside[0] - AIcp[0], AIside[1] - AIcp[1], AIside[2] - AIcp[2] },waypoints);
                    AIaccel *= 0.98;
                    
                    printf("%.3f\n", AISteerFunc);
                    if (AISteerFunc > 0.8) {
                        AICarSteerSens = 0.16;
                    }
                    else {
                        AICarSteerSens = 0.16;
                    }

                    if (AIInitAccel < 240) {
                        AIaccel += 8;
                        AIInitAccel++;
                    }

                    if (AIAccelFunc > 0) {
                        AIaccel += 7.5;
                    }
                    else if (AIAccelFunc < 0) {
                        AIaccel -= 3.5;
                    }
                    if (AISteerFunc < -1.7) {
                        AIaccel -= 0.6;
                    }
                    else if (AISteerFunc > 1.7) {
                        AIaccel -= 0.2;
                    }
                    if (AIaccel < 10.0f) {
                        AIaccel = 10.0f;
                    }
                    //printf("%d", IsAIAccel((Vector3) { AIcp[0], AIcp[1], AIcp[2] }, (Vector3) { AIcp[0] - AIco[0], AIcp[1] - AIco[1], AIcp[2] - AIco[2] }));
                    if (AIaccel > 50) AIaccel = 50;
                    if (AIaccel < -15) AIaccel = -15;
                    AIsteer *= 0.85;
                    if (AISteerFunc > 0) {
                       // AIsteer = AISteerFunc*AICarSteerSens;//-
                        AIsteer = AISteerFunc;
                    }
                    else if (AISteerFunc< 0) {
                       // AIsteer = -AISteerFunc*AICarSteerSens;//+
                        AIsteer = AISteerFunc;
                    }
                    if (AIsteer > .5) AIsteer = 0.5;
                    if (AIsteer < -.5) AIsteer = -0.5;

                    if (CheckCollisionPointRec(mousePoint, (Rectangle) { 10, 10, 80, 40 })) {
                        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
                            currentScreen = MENU;
                            menuRMAIback = 1;
                        }
                    }

                    updateVehicle(AIcar, AIaccel, 800.0, AIsteer, 10.0);

                    const double* v = dBodyGetLinearVel(carAIM->bodies[0]);
                    float vel = Vector3Length((Vector3) { v[0], v[1], v[2] }) * 2.23693629f;
                    updateVehicle(carAIM, accel, 800.0, steer, 10.0);
                    speedoac = 40 - ((240.0) / (75)) * (accel - 75);
                    speedov = 40 - ((240.0) / (85)) * (vel - 85);

                    const dReal* cp = dBodyGetPosition(carAIM->bodies[0]);
                    camera.target = (Vector3){ cp[0],cp[1] + 1,cp[2] };

                    float lerp = 0.1f;

                    dVector3 co;
                    dBodyGetRelPointPos(carAIM->bodies[0], -8, 3, 0, co);
                    dVector3 lightposL;
                    dBodyGetRelPointPos(carAIM->bodies[0], -2.3, 0.05, -0.5, lightposL);
                    dVector3 odeco;
                    dReal* rotmat = dBodyGetRotation(carAIM->bodies[0]);
                    /*printf("Line 1 : %.2f %.2f %.2f %.2f \n", rotmat[0], rotmat[1], rotmat[2], rotmat[3]);
                    printf("Line 2 : %.2f %.2f %.2f %.2f \n", rotmat[4], rotmat[5], rotmat[6], rotmat[7]);
                    printf("Line 3 : %.2f %.2f %.2f %.2f \n", rotmat[8], rotmat[9], rotmat[10], rotmat[11]);
                    printf("Line 4 : %.2f %.2f %.2f %.2f \n", rotmat[12], rotmat[13], rotmat[14], rotmat[15]);*/




                    //printf("%.3f %.3f %.3f \n", cp[0], cp[1], cp[2]);
                    /*if (IsKeyReleased(KEY_P)) {
                        fprintf(fptr, "%.3f %.3f %.3f \n", cp[0], cp[1], cp[2]);
                    }*/


                    mouseDelta = GetMouseDelta();
                    yaw += mouseDelta.x * sensitivity;
                    pitchcar += mouseDelta.y * sensitivity;
                    /*if (IsKeyDown(KEY_A)) {
                        yaw -= 0.8;
                    }
                    else if (IsKeyDown(KEY_D)) {
                        yaw += 0.8;
                    }*/


                    if (IsKeyReleased(KEY_S)) {
                        dBodySetPosition(AIcar->bodies[0], 124.2, -5.9, 206);
                    }





                    if (pitchcar > 89.0f) {
                        pitchcar = 89.0f;
                    }
                    if (pitchcar < -5.0f) {
                        pitchcar = -5.0f;
                    }

                    mousePos.x = cosf(DEG2RAD * yaw) * cosf(DEG2RAD * pitchcar);
                    mousePos.y = sin(DEG2RAD * pitchcar);
                    mousePos.z = sinf(DEG2RAD * yaw) * cosf(DEG2RAD * pitchcar);

                    if (accel > 0 && (mouseDelta.x == 0) && (mouseDelta.y == 0)) {
                        camera.position.x -= (camera.position.x - co[0]) * lerp + 0.0 * mousePos.x;// * (1/ft);
                        camera.position.y -= (camera.position.y - co[1]) * lerp + 0.0 * mousePos.y;// * (1/ft);
                        camera.position.z -= (camera.position.z - co[2]) * lerp + 0.0 * mousePos.z;// * (1/ft);
                    }
                    else {
                        camera.position.x -= (camera.position.x - (camera.target.x + 8.0 * mousePos.x)) * lerp;
                        camera.position.y -= (camera.position.y - (camera.target.y + 8.0 * mousePos.y)) * lerp;
                        camera.position.z -= (camera.position.z - (camera.target.z + 8.0 * mousePos.z)) * lerp;
                    }


                    UpdateCamera(&camera);

                    mousePos.x = mousePos.x * 0.8;
                    mousePos.y = mousePos.y * 0.8;
                    mousePos.z = mousePos.z * 0.8;


                    bool spcdn = IsKeyDown(KEY_SPACE);

                    
                    if (cp[1] < -30) {
                        dBodySetPosition(carAIM->bodies[0], 130, 1, 190);
                    }
                    if (AIcp[1] < -30) {
                        dBodySetPosition(AIcar->bodies[0], 124.2, -5.9, 206);
                    }


                    UpdateCamera(&camera);              // Update camera

                    if (IsKeyPressed(KEY_L)) { lights[0].enabled = !lights[0].enabled; UpdateLightValues(shader, lights[0]); }

                    // update the light shader with the camera view position
                    SetShaderValue(shader, shader.locs[SHADER_LOC_VECTOR_VIEW], &camera.position.x, SHADER_UNIFORM_VEC3);

                    frameTime += GetFrameTime();
                    int pSteps = 0;
                    physTime = GetTime();

                    while (frameTime > physSlice) {                       
                        dSpaceCollide(spaceAI, 0, &nearCallback);

                        dWorldQuickStep(world, physSlice);  // NB fixed time step is important
                        dJointGroupEmpty(contactgroup);

                        frameTime -= physSlice;
                        pSteps++;
                        if (pSteps > maxPsteps) {
                            frameTime = 0;
                            break;
                        }
                    }

                    physTime = GetTime() - physTime;






                    BeginDrawing();

                    ClearBackground(RAYWHITE);
                    BeginBlendMode(BLEND_ALPHA);
                    BeginMode3D(camera);
                    //BeginShaderMode(alphadiscard);
                    rlDisableBackfaceCulling();
                    rlDisableDepthMask();
                    DrawModel(skybox, (Vector3) { 0, 0, 0 }, 1.0f, WHITE);
                    rlEnableBackfaceCulling();
                    rlEnableDepthMask();
                    DrawModel(ground, (Vector3) { 0, 0, 0 }, 1, WHITE);
                    for (int i = 0; i < 174; i++) {
                        DrawSphere(waypoints[i], 0.3, BLUE);
                    }
                    //drawAllSpaceGeoms(spaceAI,carAIM);
                    drawAllSpaceTWOPGeoms(spaceAI, carAIM, AIcar, MARUSSIA, BMW);
                    EndMode3D();
                    EndBlendMode();

                    if (pSteps > maxPsteps) DrawText("WARNING CPU overloaded lagging real time", 10, 0, 20, RED);
                    DrawText(TextFormat("%2i FPS", GetFPS()), 10, 20, 20, BLACK);
                    //DrawText(TextFormat("accel %4.4f", accel), 10, 40, 20, BLACK);
                    //DrawText(TextFormat("steer %4.4f", steer), 10, 60, 20, BLACK);
                    if (!antiSway) DrawText("Anti sway bars OFF", 10, 80, 20, RED);
                    //DrawText(TextFormat("debug %4.4f %4.4f %4.4f", debug.x, debug.y, debug.z), 10, 100, 20, BLACK);
                    //DrawText(TextFormat("Phys steps per frame %i", pSteps), 10, 120, 20, BLACK);
                    //DrawText(TextFormat("Phys time per frame %i", physTime), 10, 140, 20, BLACK);
                    //DrawText(TextFormat("total time per frame %i", frameTime), 10, 160, 20, BLACK);
                    //DrawText(TextFormat("objects %i", numObj), 10, 180, 20, BLACK);
                    DrawRing((Vector2) { screenWidth - 150, screenHeight - 170 }, 100.0f, 112.0f, 40, 280, 0, Fade(WHITE, 0.6f));
                    DrawRing((Vector2) { screenWidth - 150, screenHeight - 170 }, 102.0f, 110.0f, speedoac, 280, 0, Fade(RED, 0.6f));
                    DrawRing((Vector2) { screenWidth - 150, screenHeight - 170 }, 86.0f, 96.0f, 40, 280, 0, Fade(LIGHTGRAY, 0.8f));
                    DrawRing((Vector2) { screenWidth - 150, screenHeight - 170 }, 88.0f, 94.0f, speedov, 280, 0, Fade(DARKBLUE, 0.8f));
                    DrawText(TextFormat("%.0f MPH", vel), screenWidth - 200, screenHeight - 190, 30, VIOLET);

                    //DrawText(TextFormat("roll %.4f", fabs(roll)), 10, 200, 20, BLACK);



                    //DrawText(TextFormat("mph %.4f", vel), 10, 220, 20, BLACK);
                    DrawRectangleRounded((Rectangle) { 10, 10, 80, 40 }, 0.8, 0, Fade(BLUE, 0.8));

                    EndDrawing();
                    //----------------------------------------------------------------------------------

                }break;
                case FREERIDE:
                {

                    if (IsRMCarCreatedFRM == 0) {
                        car = CreateVehicle(space, world, 130, -5.5, 200,AMwheeloff);
                        IsRMCarCreatedFRM++;
                    }

                    mousePoint = GetMousePosition();

                    SetShaderValue(l_shader, l_shader.locs[SHADER_LOC_VECTOR_VIEW],(float[3]) { camera.position.x,camera.position.y,camera.position.z}, RL_SHADER_UNIFORM_VEC3);

                    const dReal* q = dBodyGetQuaternion(car->bodies[0]);
                    double z0 = 2.0f * (q[0] * q[3] + q[1] * q[2]);
                    double z1 = 1.0f - 2.0f * (q[1] * q[1] + q[3] * q[3]);
                    double roll = atan2f(z0, z1);
                    if (fabs(roll) > (M_PI_2 - 0.001)) {
                        carFlipped++;
                    }
                    else {
                        carFlipped = 0;
                    }

                    if (carFlipped > 100) {
                        unflipVehicle(car);
                    }

                    accel *= .99;
                    if (IsKeyDown(KEY_UP)) accel += 2;
                    if (IsKeyDown(KEY_DOWN)) accel -= 2;
                    if (accel > 70) accel = 70;
                    if (accel < -15) accel = -15;

                    if (IsKeyDown(KEY_F)) {
                        ToggleFullscreen();
                    }

                    if (IsKeyDown(KEY_RIGHT)) steer -= .005;
                    if (IsKeyDown(KEY_LEFT)) steer += .005;
                    if (!IsKeyDown(KEY_RIGHT) && !IsKeyDown(KEY_LEFT)) steer *= .5;
                    if (steer > .5) steer = .5;
                    if (steer < -.5) steer = -.5;

                    if (CheckCollisionPointRec(mousePoint, (Rectangle) { 10, 10, 80, 40 })) {
                        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
                            currentScreen = MENU;
                            menuFRback = 1;
                        }
                    }

                    const double* v = dBodyGetLinearVel(car->bodies[0]);
                    float vel = Vector3Length((Vector3) { v[0], v[1], v[2] }) * 2.23693629f;
                    updateVehicle(car, accel, 800.0, steer, 10.0);
                    speedoac = 40-((240.0)/(75)) * (accel - 75);
                    speedov = 40 - ((240.0)/(85))* (vel - 85);

                    const dReal* cp = dBodyGetPosition(car->bodies[0]);
                    camera.target = (Vector3){ cp[0],cp[1] + 1,cp[2] };

                    float lerp = 0.1f;

                    dVector3 co;
                    dBodyGetRelPointPos(car->bodies[0], -8, 3, 0, co);
                    dVector3 lightposL;
                    dBodyGetRelPointPos(car->bodies[0], -2.3, 0.05, -0.5, lightposL);
                    dVector3 odeco;
                    dReal* rotmat = dBodyGetRotation(car->bodies[0]);
                    /*printf("Line 1 : %.2f %.2f %.2f %.2f \n", rotmat[0], rotmat[1], rotmat[2], rotmat[3]);
                    printf("Line 2 : %.2f %.2f %.2f %.2f \n", rotmat[4], rotmat[5], rotmat[6], rotmat[7]);
                    printf("Line 3 : %.2f %.2f %.2f %.2f \n", rotmat[8], rotmat[9], rotmat[10], rotmat[11]);
                    printf("Line 4 : %.2f %.2f %.2f %.2f \n", rotmat[12], rotmat[13], rotmat[14], rotmat[15]);*/
                    


                    
                    printf("%.3f %.3f %.3f \n", cp[0], cp[1], cp[2]);
                    /*if (IsKeyReleased(KEY_P)) {
                        fprintf(chptr, "%.3f %.3f %.3f \n", cp[0], cp[1], cp[2]);
                    }*/
                    //printf("%.3f %.3f %.3f \n", camera.position.x, camera.position.y, camera.position.z);
          
                    
                    mouseDelta = GetMouseDelta();
                    yaw += mouseDelta.x*sensitivity;
                    pitchcar += mouseDelta.y*sensitivity;
                    /*if (IsKeyDown(KEY_A)) {
                        yaw -= 0.8;
                    }
                    else if (IsKeyDown(KEY_D)) {
                        yaw += 0.8;
                    }*/
                    

                    

                    
                    
                    

                    if (pitchcar > 89.0f) {
                        pitchcar = 89.0f;
                    }
                    if (pitchcar < -5.0f) {
                        pitchcar = -5.0f;
                    }

                    mousePos.x = cosf(DEG2RAD * yaw) * cosf(DEG2RAD * pitchcar);
                    mousePos.y = sin(DEG2RAD * pitchcar);
                    mousePos.z = sinf(DEG2RAD * yaw)*cosf(DEG2RAD * pitchcar);
                    if (accel > 0 && (mouseDelta.x==0) && (mouseDelta.y==0)) {
                        camera.position.x -= (camera.position.x - co[0]) * lerp + 0.0 * mousePos.x;// * (1/ft);
                        camera.position.y -= (camera.position.y - co[1]) * lerp + 0.0 * mousePos.y;// * (1/ft);
                        camera.position.z -= (camera.position.z - co[2]) * lerp + 0.0 * mousePos.z;// * (1/ft);
                    }
                    else {
                        camera.position.x -=(camera.position.x-(camera.target.x + 8.0 * mousePos.x))*lerp;
                        camera.position.y -=(camera.position.y-(camera.target.y + 8.0 * mousePos.y))*lerp;
                        camera.position.z -=(camera.position.z-(camera.target.z + 8.0 * mousePos.z))*lerp;
                    }


                    UpdateCamera(&camera);

                    mousePos.x = mousePos.x * 0.8;
                    mousePos.y = mousePos.y * 0.8;
                    mousePos.z = mousePos.z * 0.8;


                    bool spcdn = IsKeyDown(KEY_SPACE);

                    for (int i = 0; i < numObj; i++) {
                        const dReal* pos = dBodyGetPosition(obj[i]);
                        if (spcdn) {
                            // apply force if the space key is held
                            const dReal* v = dBodyGetLinearVel(obj[0]);
                            if (v[1] < 10 && pos[1] < 10) { // cap upwards velocity and don't let it get too high
                                dBodyEnable(obj[i]); // case its gone to sleep
                                dMass mass;
                                dBodyGetMass(obj[i], &mass);
                                // give some object more force than others
                                float f = (6 + (((float)i / numObj) * 4)) * mass.mass;
                                dBodyAddForce(obj[i], rndf(-f, f), f * 4, rndf(-f, f));
                            }
                        }


                        if (pos[1] < -10) {
                            // teleport back if fallen off the ground
                            dBodySetPosition(obj[i], dRandReal() * 10 - 5,
                                12 + rndf(1, 2), dRandReal() * 10 - 5);
                            dBodySetLinearVel(obj[i], 0, 0, 0);
                            dBodySetAngularVel(obj[i], 0, 0, 0);
                        }
                    }
                    if (cp[1] < -30) {
                        dBodySetPosition(car->bodies[0], 130, 1, 190);
                    }
                    


                    UpdateCamera(&camera);              // Update camera

                    if (IsKeyPressed(KEY_L)) { lights[0].enabled = !lights[0].enabled; UpdateLightValues(shader, lights[0]); }

                    // update the light shader with the camera view position
                    SetShaderValue(shader, shader.locs[SHADER_LOC_VECTOR_VIEW], &camera.position.x, SHADER_UNIFORM_VEC3);

                    frameTime += GetFrameTime();
                    int pSteps = 0;
                    physTime = GetTime();

                    while (frameTime > physSlice) {
                        // check for collisions
                        // TODO use 2nd param data to pass custom structure with
                        // world and space ID's to avoid use of globals...
                        dSpaceCollide(space, 0, &nearCallback);

                        // step the world
                        dWorldQuickStep(world, physSlice);  // NB fixed time step is important
                        dJointGroupEmpty(contactgroup);

                        frameTime -= physSlice;
                        pSteps++;
                        if (pSteps > maxPsteps) {
                            frameTime = 0;
                            break;
                        }
                    }

                    physTime = GetTime() - physTime;






                    BeginDrawing();

                    ClearBackground(RAYWHITE);
                    BeginBlendMode(BLEND_ALPHA);
                    BeginMode3D(camera);
                    //BeginShaderMode(alphadiscard);
                    rlDisableBackfaceCulling();
                    rlDisableDepthMask();
                    DrawModel(skybox, (Vector3) { 0, 0, 0 }, 1.0f, WHITE);
                    rlEnableBackfaceCulling();
                    rlEnableDepthMask();
                    DrawModel(ground, (Vector3) { 0, 0, 0 }, 1, WHITE);
                    
                    drawAllSpaceGeoms(space,car);
                    //DrawGrid(100, 1.0f);


                    //DrawPlane((Vector3){ 0.0f, 0.0f, 0.0f }, (Vector2){ 32.0f, 32.0f }, LIGHTGRAY); // Draw ground
                    //DrawCube((Vector3){ -16.0f, 2.5f, 0.0f }, 1.0f, 5.0f, 32.0f, BLUE);     // Draw a blue wall
                    //DrawCube((Vector3){ 16.0f, 2.5f, 0.0f }, 1.0f, 5.0f, 32.0f, LIME);      // Draw a green wall
                    //DrawCube((Vector3){ 0.0f, 2.5f, 16.0f }, 32.0f, 5.0f, 1.0f, GOLD);      // Draw a yellow ball

                   //EndShaderMode();
                    EndMode3D();
                    EndBlendMode();

                    if (pSteps > maxPsteps) DrawText("WARNING CPU overloaded lagging real time", 10, 0, 20, RED);
                    DrawText(TextFormat("%2i FPS", GetFPS()), 10, 20, 20, BLACK);
                    //DrawText(TextFormat("accel %4.4f", accel), 10, 40, 20, BLACK);
                    //DrawText(TextFormat("steer %4.4f", steer), 10, 60, 20, BLACK);
                    if (!antiSway) DrawText("Anti sway bars OFF", 10, 80, 20, RED);
                    //DrawText(TextFormat("debug %4.4f %4.4f %4.4f", debug.x, debug.y, debug.z), 10, 100, 20, BLACK);
                    //DrawText(TextFormat("Phys steps per frame %i", pSteps), 10, 120, 20, BLACK);
                    //DrawText(TextFormat("Phys time per frame %i", physTime), 10, 140, 20, BLACK);
                    //DrawText(TextFormat("total time per frame %i", frameTime), 10, 160, 20, BLACK);
                    //DrawText(TextFormat("objects %i", numObj), 10, 180, 20, BLACK);
                    DrawRing((Vector2) { screenWidth - 150, screenHeight - 170 }, 100.0f, 112.0f, 40, 280, 0, Fade(WHITE, 0.6f));
                    DrawRing((Vector2) { screenWidth - 150, screenHeight - 170 }, 102.0f, 110.0f, speedoac, 280, 0, Fade(RED, 0.6f));
                    DrawRing((Vector2) { screenWidth - 150, screenHeight - 170 }, 86.0f, 96.0f, 40, 280, 0, Fade(LIGHTGRAY, 0.8f));
                    DrawRing((Vector2) { screenWidth - 150, screenHeight - 170 }, 88.0f, 94.0f, speedov, 280, 0, Fade(DARKBLUE, 0.8f));
                    DrawText(TextFormat("%.0f MPH", vel), screenWidth-200, screenHeight-190, 30, VIOLET);

                    //DrawText(TextFormat("roll %.4f", fabs(roll)), 10, 200, 20, BLACK);

                    
                    
                    //DrawText(TextFormat("mph %.4f", vel), 10, 220, 20, BLACK);
                    DrawRectangleRounded((Rectangle) { 10, 10, 80, 40 }, 0.8, 0, Fade(BLUE, 0.8));
                    EndDrawing();
                    //----------------------------------------------------------------------------------
                }break;
                case OPTIONS:
                {

                }break;
                
            
        }


    }

    UnloadMusicStream(music);          // Unload music stream buffers from RAM

    CloseAudioDevice();


    UnloadModel(box);
    UnloadModel(ball);
    UnloadModel(cylinder);
    UnloadModel(ground);
    UnloadTexture(drumTx);
    UnloadTexture(earthTx);
    UnloadTexture(crateTx);
    UnloadTexture(grassTx);
    UnloadShader(shader);

    RL_FREE(car);

    RL_FREE(groundInd);
    dGeomTriMeshDataDestroy(triData);

    dJointGroupEmpty(contactgroup);
    dJointGroupDestroy(contactgroup);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();

    CloseWindow();        
    

    return 0;
}