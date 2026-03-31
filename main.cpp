#define _CRT_SECURE_NO_WARNINGS
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include <freeglut.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <time.h>

// ===== C1+C2: Include =====
#include "car.h"
#include "pedestrian.h"
#include "collision.h"
#include "ambulance.h"

//CONSTANTE 
#define TERRAIN_SIZE    256
#define TERRAIN_SCALE   1.0f
#define TERRAIN_HEIGHT  6.0f
#define CUBE_SIZE       120.0f
#ifndef PI
#define PI              3.14159265f
#endif
#define ROAD_WIDTH      5.5f
#define MAX_LIGHTS      8  

//STRUCTURI 
struct Vec2 { float x, z; };
// LightPos e definit in collision.h  

// CIRCUIT - puncte de control 
static Vec2 trackPoints[] = {
    { 50, -10}, { 50,  10},
    { 50,  30}, { 40,  45}, { 25,  52}, { 10,  52},
    {-10,  50}, {-20,  42}, {-15,  32},
    {-25,  25}, {-45,  20}, {-55,   5}, {-55, -10},
    {-55, -25}, {-45, -40}, {-30, -50}, {-10, -55},
    { 10, -55}, { 30, -55}, { 48, -48},
    { 55, -35}, { 55, -20}, { 50, -10}
};
static int numTrackPoints = sizeof(trackPoints) / sizeof(trackPoints[0]);

// VARIABILE GLOBALE 

GLuint texGrass, texSkyFront, texSkyBack, texSkyLeft, texSkyRight, texSkyTop, texSkyBottom;
GLuint texRoad;


float camX = 0.0f, camY = 8.0f, camZ = 30.0f;
float camYaw = 0.0f, camPitch = -15.0f;
float camDist = 1.0f;
bool  firstPerson = false;

int   lastMouseX = -1, lastMouseY = -1;
bool  leftDown = false, rightDown = false;
int   winW = 1280, winH = 720;


bool  nightMode = false;
float sunDir[4] = { 0.6f,1.0f,0.4f,0.0f };


std::vector<LightPos> polePositions;


float heightmap[TERRAIN_SIZE][TERRAIN_SIZE];


std::vector<Vec2> trackSpline;

// ===== C1+C2: Variabile =====
Car playerCar;
#define NUM_PEDESTRIANS 5
Pedestrian pedestrians[NUM_PEDESTRIANS];
std::vector<AABB> buildingBoxes;
std::vector<KnockableCone> knockableCones;
std::vector<CircleCollider> poleColliders;

// C2: Ambulanta
#define NUM_AMBULANCES 2
Ambulance ambulances[NUM_AMBULANCES];

// Taste sageti (separate de WASD)
bool keyArrowUp = false, keyArrowDown = false;
bool keyArrowLeft = false, keyArrowRight = false;

// Timing
float lastTime = 0.0f;
float getDeltaTime() {
    float current = glutGet(GLUT_ELAPSED_TIME) / 1000.0f;
    float dt = current - lastTime;
    lastTime = current;
    if (dt > 0.1f) dt = 0.1f;  // cap la 100ms
    if (dt < 0.0f) dt = 0.016f;
    return dt;
}

// Camera follow mode
bool cameraFollow = false;  // C toggle

// Lap counter
int   lapCount = 0;
bool  crossedStart = false;   // previne dubla contorizare
int   lapTarget = 3;
bool  raceWon = false;
float winTimer = 0.0f;        // timer afisare mesaj victorie

//INTERPOLARE CATMULL-ROM 
Vec2 catmullRom(Vec2 p0, Vec2 p1, Vec2 p2, Vec2 p3, float t) {
    float t2 = t * t, t3 = t2 * t;
    Vec2 r;
    r.x = 0.5f * ((2 * p1.x) + (-p0.x + p2.x) * t + (2 * p0.x - 5 * p1.x + 4 * p2.x - p3.x) * t2 + (-p0.x + 3 * p1.x - 3 * p2.x + p3.x) * t3);
    r.z = 0.5f * ((2 * p1.z) + (-p0.z + p2.z) * t + (2 * p0.z - 5 * p1.z + 4 * p2.z - p3.z) * t2 + (-p0.z + 3 * p1.z - 3 * p2.z + p3.z) * t3);
    return r;
}
std::vector<Vec2> buildTrackSpline(int steps = 20) {
    std::vector<Vec2> pts;
    int n = numTrackPoints;
    for (int i = 0; i < n; i++) {
        Vec2 p0 = trackPoints[(i - 1 + n) % n], p1 = trackPoints[i];
        Vec2 p2 = trackPoints[(i + 1) % n], p3 = trackPoints[(i + 2) % n];
        for (int s = 0; s < steps; s++) {
            float t = (float)s / steps;
            pts.push_back(catmullRom(p0, p1, p2, p3, t));
        }
    }
    return pts;
}

//GENERARE TEREN 
float cosInterp(float a, float b, float t) { float f = (1.0f - cosf(t * PI)) * 0.5f; return a * (1 - f) + b * f; }
float pseudoRand(int x, int y) { int n = x + y * 57; n = (n << 13) ^ n; return(1.0f - ((n * (n * n * 15731 + 789221) + 1376312589) & 0x7fffffff) / 1073741824.0f); }
float smoothNoise(float x, float y) { int ix = (int)x, iy = (int)y; float fx = x - ix, fy = y - iy; return cosInterp(cosInterp(pseudoRand(ix, iy), pseudoRand(ix + 1, iy), fx), cosInterp(pseudoRand(ix, iy + 1), pseudoRand(ix + 1, iy + 1), fx), fy); }

void generateTerrain() {
    for (int z = 0; z < TERRAIN_SIZE; z++)for (int x = 0; x < TERRAIN_SIZE; x++) {
        float nx = x * 0.04f, nz = z * 0.04f;
        float h = smoothNoise(nx, nz) + smoothNoise(nx * 2, nz * 2) * 0.5f + smoothNoise(nx * 4, nz * 4) * 0.25f + smoothNoise(nx * 8, nz * 8) * 0.125f;
        h /= 1.875f;
        float dx = (float)x / TERRAIN_SIZE - 0.5f, dz2 = (float)z / TERRAIN_SIZE - 0.5f;
        float dist = sqrtf(dx * dx + dz2 * dz2) * 2.0f;
        heightmap[z][x] = h * fmaxf(0.0f, 1.0f - dist) * TERRAIN_HEIGHT;
    }
    for (int z = 0; z < TERRAIN_SIZE; z++)for (int x = 0; x < TERRAIN_SIZE; x++) {
        float wx = (x - TERRAIN_SIZE * 0.5f) * TERRAIN_SCALE;
        float wz = (z - TERRAIN_SIZE * 0.5f) * TERRAIN_SCALE;
        float minD = 9999.0f;
        for (auto& p : trackSpline) { float dx = wx - p.x, dz = wz - p.z; float d = sqrtf(dx * dx + dz * dz); if (d < minD)minD = d; }
        float flat = ROAD_WIDTH + 3.0f, blend = ROAD_WIDTH + 8.0f;
        if (minD < flat) heightmap[z][x] = 0.0f;
        else if (minD < blend) { float t = (minD - flat) / (blend - flat); heightmap[z][x] *= t * t; }
    }
}
float getHeight(float wx, float wz) {
    float hx = wx / TERRAIN_SCALE + TERRAIN_SIZE * 0.5f, hz = wz / TERRAIN_SCALE + TERRAIN_SIZE * 0.5f;
    int ix = (int)hx, iz = (int)hz;
    if (ix < 0)ix = 0; if (iz < 0)iz = 0; if (ix >= TERRAIN_SIZE - 1)ix = TERRAIN_SIZE - 2; if (iz >= TERRAIN_SIZE - 1)iz = TERRAIN_SIZE - 2;
    float fx = hx - ix, fz = hz - iz;
    return heightmap[iz][ix] * (1 - fx) * (1 - fz) + heightmap[iz][ix + 1] * fx * (1 - fz) + heightmap[iz + 1][ix] * (1 - fx) * fz + heightmap[iz + 1][ix + 1] * fx * fz;
}

//TEXTURA 
GLuint loadTexture(const char* fn, unsigned char fr = 80, unsigned char fg = 80, unsigned char fb = 80) {
    int w, h, c; stbi_set_flip_vertically_on_load(1);
    unsigned char* data = stbi_load(fn, &w, &h, &c, 3);
    GLuint tid; glGenTextures(1, &tid); glBindTexture(GL_TEXTURE_2D, tid);
    if (!data) { printf("EROARE: %s\n", fn); unsigned char col[3] = { fr,fg,fb }; glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1, 1, 0, GL_RGB, GL_UNSIGNED_BYTE, col); }
    else { glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, data); stbi_image_free(data); printf("OK: %s\n", fn); }
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    return tid;
}

// SHADOW MATRIX 
void shadowMatrix(float m[16], float lx, float ly, float lz, float lw, float py = 0.01f) {
    float nx = 0, ny = 1, nz = 0, d = -py;
    float dot = nx * lx + ny * ly + nz * lz + d * lw;
    m[0] = dot - lx * nx; m[4] = -lx * ny; m[8] = -lx * nz; m[12] = -lx * d;
    m[1] = -ly * nx; m[5] = dot - ly * ny; m[9] = -ly * nz; m[13] = -ly * d;
    m[2] = -lz * nx; m[6] = -lz * ny; m[10] = dot - lz * nz; m[14] = -lz * d;
    m[3] = -lw * nx; m[7] = -lw * ny; m[11] = -lw * nz; m[15] = dot - lw * d;
}

//SKYBOX 
void drawSkybox() {
    float s = CUBE_SIZE;
    glDisable(GL_LIGHTING); glDisable(GL_DEPTH_TEST); glEnable(GL_TEXTURE_2D); glColor3f(1, 1, 1);
    glBindTexture(GL_TEXTURE_2D, texSkyFront);
    glBegin(GL_QUADS); glTexCoord2f(0, 0); glVertex3f(-s, -s, -s); glTexCoord2f(1, 0); glVertex3f(s, -s, -s); glTexCoord2f(1, 1); glVertex3f(s, s, -s); glTexCoord2f(0, 1); glVertex3f(-s, s, -s); glEnd();
    glBindTexture(GL_TEXTURE_2D, texSkyBack);
    glBegin(GL_QUADS); glTexCoord2f(0, 0); glVertex3f(s, -s, s); glTexCoord2f(1, 0); glVertex3f(-s, -s, s); glTexCoord2f(1, 1); glVertex3f(-s, s, s); glTexCoord2f(0, 1); glVertex3f(s, s, s); glEnd();
    glBindTexture(GL_TEXTURE_2D, texSkyLeft);
    glBegin(GL_QUADS); glTexCoord2f(0, 0); glVertex3f(-s, -s, s); glTexCoord2f(1, 0); glVertex3f(-s, -s, -s); glTexCoord2f(1, 1); glVertex3f(-s, s, -s); glTexCoord2f(0, 1); glVertex3f(-s, s, s); glEnd();
    glBindTexture(GL_TEXTURE_2D, texSkyRight);
    glBegin(GL_QUADS); glTexCoord2f(0, 0); glVertex3f(s, -s, -s); glTexCoord2f(1, 0); glVertex3f(s, -s, s); glTexCoord2f(1, 1); glVertex3f(s, s, s); glTexCoord2f(0, 1); glVertex3f(s, s, -s); glEnd();
    glBindTexture(GL_TEXTURE_2D, texSkyTop);
    glBegin(GL_QUADS); glTexCoord2f(0, 1); glVertex3f(-s, s, -s); glTexCoord2f(1, 1); glVertex3f(s, s, -s); glTexCoord2f(1, 0); glVertex3f(s, s, s); glTexCoord2f(0, 0); glVertex3f(-s, s, s); glEnd();
    glBindTexture(GL_TEXTURE_2D, texSkyBottom);
    glBegin(GL_QUADS); glTexCoord2f(0, 0); glVertex3f(-s, -s, s); glTexCoord2f(1, 0); glVertex3f(s, -s, s); glTexCoord2f(1, 1); glVertex3f(s, -s, -s); glTexCoord2f(0, 1); glVertex3f(-s, -s, -s); glEnd();
    glEnable(GL_DEPTH_TEST); glDisable(GL_TEXTURE_2D);
}

//TEREN 
void drawTerrain() {
    glEnable(GL_TEXTURE_2D); glBindTexture(GL_TEXTURE_2D, texGrass);
    glColor3f(1, 1, 1); glEnable(GL_LIGHTING); glEnable(GL_LIGHT0);
    float ts = 0.08f;
    for (int z = 0; z < TERRAIN_SIZE - 1; z++) {
        glBegin(GL_TRIANGLE_STRIP);
        for (int x = 0; x < TERRAIN_SIZE; x++) {
            float hL = (x > 0) ? heightmap[z][x - 1] : heightmap[z][x];
            float hR = (x < TERRAIN_SIZE - 1) ? heightmap[z][x + 1] : heightmap[z][x];
            float hD = (z > 0) ? heightmap[z - 1][x] : heightmap[z][x];
            float hU = (z < TERRAIN_SIZE - 1) ? heightmap[z + 1][x] : heightmap[z][x];
            float nx = hL - hR, ny = 2.0f * TERRAIN_SCALE, nz2 = hD - hU;
            float len = sqrtf(nx * nx + ny * ny + nz2 * nz2);
            glNormal3f(nx / len, ny / len, nz2 / len);
            float wx = (x - TERRAIN_SIZE * 0.5f) * TERRAIN_SCALE;
            float wz0 = (z - TERRAIN_SIZE * 0.5f) * TERRAIN_SCALE;
            float wz1 = ((z + 1) - TERRAIN_SIZE * 0.5f) * TERRAIN_SCALE;
            glTexCoord2f(x * ts, z * ts);    glVertex3f(wx, heightmap[z][x], wz0);
            glTexCoord2f(x * ts, (z + 1) * ts); glVertex3f(wx, heightmap[z + 1][x], wz1);
        }
        glEnd();
    }
    glDisable(GL_TEXTURE_2D);
}

//CIRCUIT 
void drawCircuit() {
    glEnable(GL_TEXTURE_2D); glBindTexture(GL_TEXTURE_2D, texRoad);
    glDisable(GL_LIGHTING); glColor3f(1, 1, 1);
    int n = (int)trackSpline.size();
    float texV = 0.0f;
    glBegin(GL_TRIANGLE_STRIP);
    for (int i = 0; i <= n; i++) {
        Vec2 cur = trackSpline[i % n], nxt = trackSpline[(i + 1) % n];
        float tx = nxt.x - cur.x, tz = nxt.z - cur.z;
        float len = sqrtf(tx * tx + tz * tz); if (len < 0.0001f)len = 0.0001f;
        tx /= len; tz /= len;
        float nx2 = tz, nz2 = -tx;
        glTexCoord2f(0.0f, texV); glVertex3f(cur.x - nx2 * ROAD_WIDTH, 0.06f, cur.z - nz2 * ROAD_WIDTH);
        glTexCoord2f(1.0f, texV); glVertex3f(cur.x + nx2 * ROAD_WIDTH, 0.06f, cur.z + nz2 * ROAD_WIDTH);
        texV += 0.12f;
    }
    glEnd();
    glEnable(GL_LIGHTING); glDisable(GL_TEXTURE_2D);
}

void drawStartLine() {
    glDisable(GL_TEXTURE_2D); glDisable(GL_LIGHTING);
    Vec2 c = trackSpline[0];
    for (int i = 0; i < 5; i++) {
        float t0 = -ROAD_WIDTH + i * (ROAD_WIDTH * 2 / 5.0f);
        float t1 = t0 + ROAD_WIDTH * 2 / 5.0f * 0.5f;
        if (i % 2 == 0)glColor3f(1, 1, 1); else glColor3f(0, 0, 0);
        glBegin(GL_QUADS);
        glVertex3f(c.x + t0, 0.12f, c.z - 1.0f); glVertex3f(c.x + t1, 0.12f, c.z - 1.0f);
        glVertex3f(c.x + t1, 0.12f, c.z + 1.0f); glVertex3f(c.x + t0, 0.12f, c.z + 1.0f);
        glEnd();
    }
    glEnable(GL_LIGHTING);
}

void drawBarriers() {
    glDisable(GL_TEXTURE_2D); glEnable(GL_LIGHTING);
    int n = (int)trackSpline.size();
    for (int side = 0; side < 2; side++) {
        float ss = (side == 0) ? 1.0f : -1.0f, off = ROAD_WIDTH + 0.5f;
        for (int i = 0; i < n; i++) {
            Vec2 cur = trackSpline[i], nxt = trackSpline[(i + 1) % n];
            float tx = nxt.x - cur.x, tz = nxt.z - cur.z;
            float len = sqrtf(tx * tx + tz * tz); if (len < 0.001f)len = 0.001f;
            tx /= len; tz /= len;
            float nx2 = tz * ss, nz2 = -tx * ss;
            Vec2 c1 = { cur.x + nx2 * off,cur.z + nz2 * off }, c2 = { nxt.x + nx2 * off,nxt.z + nz2 * off };
            if ((i / 3) % 2 == 0)glColor3f(1.0f, 0.15f, 0.15f); else glColor3f(1.0f, 1.0f, 1.0f);
            glBegin(GL_QUADS);
            glNormal3f(nx2, 0, nz2);
            glVertex3f(c1.x, 0.0f, c1.z); glVertex3f(c2.x, 0.0f, c2.z);
            glVertex3f(c2.x, 0.7f, c2.z); glVertex3f(c1.x, 0.7f, c1.z);
            glEnd();
        }
    }
    glColor3f(1, 1, 1);
}

// OBIECTE STATICE
void drawTree(float x, float z, float h, float v) {
    glDisable(GL_TEXTURE_2D);
    glColor3f(0.35f + v * 0.1f, 0.22f, 0.08f);
    float tr = 0.25f, th = h * 0.38f;
    glBegin(GL_TRIANGLE_STRIP);
    for (int i = 0; i <= 10; i++) { float a = (float)i / 10 * 2 * PI; glNormal3f(cosf(a), 0, sinf(a)); glVertex3f(x + cosf(a) * tr, 0, z + sinf(a) * tr); glVertex3f(x + cosf(a) * tr * 0.6f, th, z + sinf(a) * tr * 0.6f); }
    glEnd();
    float cy = th, cr = h * 0.38f;
    for (int c = 0; c < 4; c++) {
        float bR = cr * (1.0f - c * 0.18f), topY = cy + h * 0.28f, botY = cy - c * h * 0.07f;
        glColor3f(0.08f + v * 0.06f, 0.48f + v * 0.1f - c * 0.04f, 0.08f);
        glBegin(GL_TRIANGLE_FAN);
        glNormal3f(0, 1, 0); glVertex3f(x, topY, z);
        for (int i = 0; i <= 12; i++) { float a = (float)i / 12 * 2 * PI; glNormal3f(cosf(a), 0.4f, sinf(a)); glVertex3f(x + cosf(a) * bR, botY, z + sinf(a) * bR); }
        glEnd();
        cy -= h * 0.07f;
    }
}

void drawLightPoleAt(float x, float z, float rot) {
    glDisable(GL_TEXTURE_2D);
    float ph = 6.5f, pr = 0.1f;
    if (nightMode) glColor3f(0.5f, 0.5f, 0.55f);
    else          glColor3f(0.4f, 0.4f, 0.45f);
    glBegin(GL_TRIANGLE_STRIP);
    for (int i = 0; i <= 8; i++) { float a = (float)i / 8 * 2 * PI; glNormal3f(cosf(a), 0, sinf(a)); glVertex3f(x + cosf(a) * pr, 0, z + sinf(a) * pr); glVertex3f(x + cosf(a) * pr * 0.7f, ph, z + sinf(a) * pr * 0.7f); }
    glEnd();
    float brad = rot * PI / 180.0f, bx = cosf(brad) * 2.0f, bz = sinf(brad) * 2.0f;
    glColor3f(0.35f, 0.35f, 0.4f);
    glBegin(GL_LINES); glVertex3f(x, ph, z); glVertex3f(x + bx, ph + 0.3f, z + bz); glVertex3f(x + bx, ph + 0.3f, z + bz); glVertex3f(x + bx, ph - 0.5f, z + bz); glEnd();
    if (nightMode) glColor3f(1.0f, 1.0f, 0.6f);
    else          glColor3f(0.9f, 0.88f, 0.6f);
    glPushMatrix(); glTranslatef(x + bx, ph - 0.3f, z + bz); glutSolidSphere(0.35f, 8, 6); glPopMatrix();
}

void drawBuilding(float x, float z, float w, float d, float h, float r, float g, float b) {
    glDisable(GL_TEXTURE_2D); glColor3f(r, g, b);
    float verts[4][2] = { {x,z},{x + w,z},{x + w,z + d},{x,z + d} };
    float norms[4][2] = { {0,-1},{1,0},{0,1},{-1,0} };
    for (int f = 0; f < 4; f++) {
        int n1 = (f + 1) % 4;
        glBegin(GL_QUADS); glNormal3f(norms[f][0], 0, norms[f][1]);
        glVertex3f(verts[f][0], 0, verts[f][1]); glVertex3f(verts[n1][0], 0, verts[n1][1]);
        glVertex3f(verts[n1][0], h, verts[n1][1]); glVertex3f(verts[f][0], h, verts[f][1]);
        glEnd();
    }
    glColor3f(r * 0.55f, g * 0.55f, b * 0.55f);
    glBegin(GL_QUADS); glNormal3f(0, 1, 0); glVertex3f(x, h, z); glVertex3f(x + w, h, z); glVertex3f(x + w, h, z + d); glVertex3f(x, h, z + d); glEnd();
    if (nightMode) glColor3f(1.0f, 0.95f, 0.6f);
    else          glColor3f(0.5f, 0.75f, 0.95f);
    int floors = (int)(h / 2.0f), cols = (int)(w / 1.8f);
    for (int fl = 0; fl < floors; fl++)for (int col = 0; col < cols; col++) {
        float wx = x + 0.5f + col * 1.8f, wy = 0.6f + fl * 2.0f;
        glBegin(GL_QUADS); glNormal3f(0, 0, -1);
        glVertex3f(wx, wy, z - 0.01f); glVertex3f(wx + 0.9f, wy, z - 0.01f);
        glVertex3f(wx + 0.9f, wy + 1.0f, z - 0.01f); glVertex3f(wx, wy + 1.0f, z - 0.01f);
        glEnd();
    }
}

void drawTribune(float x, float z, float w, float rows, float rot) {
    glDisable(GL_TEXTURE_2D); glPushMatrix(); glTranslatef(x, 0, z); glRotatef(rot, 0, 1, 0);
    float rh = 0.6f, rd = 1.2f;
    for (int r = 0; r < (int)rows; r++) {
        float ry = r * rh * 0.7f, rz = r * rd;
        if (r % 3 == 0)glColor3f(0.85f, 0.1f, 0.1f); else if (r % 3 == 1)glColor3f(1.0f, 1.0f, 1.0f); else glColor3f(0.1f, 0.1f, 0.75f);
        glBegin(GL_QUADS); glNormal3f(0, 1, 0); glVertex3f(0, ry, rz); glVertex3f(w, ry, rz); glVertex3f(w, ry, rz + rd); glVertex3f(0, ry, rz + rd); glEnd();
        glColor3f(0.75f, 0.75f, 0.78f);
        glBegin(GL_QUADS); glNormal3f(0, 0, 1); glVertex3f(0, 0, rz); glVertex3f(w, 0, rz); glVertex3f(w, ry, rz); glVertex3f(0, ry, rz); glEnd();
    }
    float rowy = (int)rows * rh * 0.7f;
    glColor3f(0.2f, 0.2f, 0.25f);
    glBegin(GL_QUADS); glNormal3f(0, 1, 0);
    glVertex3f(-0.5f, rowy + 1.5f, -0.5f); glVertex3f(w + 0.5f, rowy + 1.5f, -0.5f);
    glVertex3f(w + 0.5f, rowy + 1.5f, (int)rows * rd + 0.5f); glVertex3f(-0.5f, rowy + 1.5f, (int)rows * rd + 0.5f);
    glEnd();
    glPopMatrix();
}

void drawCone(float x, float z) {
    glDisable(GL_TEXTURE_2D);
    glColor3f(1.0f, 0.45f, 0.0f);
    glBegin(GL_TRIANGLE_FAN); glNormal3f(0, 1, 0); glVertex3f(x, 0.75f, z);
    for (int i = 0; i <= 10; i++) { float a = (float)i / 10 * 2 * PI; glNormal3f(cosf(a), 0.5f, sinf(a)); glVertex3f(x + cosf(a) * 0.22f, 0, z + sinf(a) * 0.22f); }
    glEnd();
    glColor3f(1, 1, 1);
    glBegin(GL_TRIANGLE_STRIP);
    for (int i = 0; i <= 10; i++) { float a = (float)i / 10 * 2 * PI; glVertex3f(x + cosf(a) * 0.18f, 0.22f, z + sinf(a) * 0.18f); glVertex3f(x + cosf(a) * 0.14f, 0.30f, z + sinf(a) * 0.14f); }
    glEnd();
}

void drawBillboard(float x, float z, float rot, int cs) {
    glDisable(GL_TEXTURE_2D); glPushMatrix(); glTranslatef(x, 0, z); glRotatef(rot, 0, 1, 0);
    glColor3f(0.4f, 0.4f, 0.4f);
    for (int p = 0; p < 2; p++) {
        float px = p * 3.5f;
        glBegin(GL_TRIANGLE_STRIP);
        for (int i = 0; i <= 6; i++) { float a = (float)i / 6 * 2 * PI; glVertex3f(px + cosf(a) * 0.08f, 0, sinf(a) * 0.08f); glVertex3f(px + cosf(a) * 0.08f, 3.5f, sinf(a) * 0.08f); }
        glEnd();
    }
    float colors[3][2][3] = { {{0.9f,0.1f,0.1f},{1.0f,1.0f,0.0f}},{{0.1f,0.3f,0.8f},{1.0f,0.85f,0.0f}},{{0.1f,0.6f,0.1f},{1.0f,1.0f,1.0f}} };
    int c = cs % 3;
    glColor3f(colors[c][0][0], colors[c][0][1], colors[c][0][2]);
    glBegin(GL_QUADS); glNormal3f(0, 0, 1); glVertex3f(-0.1f, 2.2f, -0.05f); glVertex3f(3.6f, 2.2f, -0.05f); glVertex3f(3.6f, 3.8f, -0.05f); glVertex3f(-0.1f, 3.8f, -0.05f); glEnd();
    glColor3f(colors[c][1][0], colors[c][1][1], colors[c][1][2]);
    glBegin(GL_QUADS); glNormal3f(0, 0, 1); glVertex3f(0.3f, 2.7f, 0); glVertex3f(3.2f, 2.7f, 0); glVertex3f(3.2f, 3.3f, 0); glVertex3f(0.3f, 3.3f, 0); glEnd();
    glPopMatrix();
}

// DESENEAZA UMBRA UNUI OBIECT
void setShadowMatrix(float lx, float ly, float lz, float lw) {
    float m[16];
    shadowMatrix(m, lx, ly, lz, lw, 0.05f);
    glMultMatrixf(m);
}

void drawPoleShadow(float x, float z) {
    float ph = 6.5f, pr = 0.25f;
    glBegin(GL_TRIANGLE_STRIP);
    for (int i = 0; i <= 10; i++) {
        float a = (float)i / 10 * 2 * PI;
        glVertex3f(x + cosf(a) * pr, 0.05f, z + sinf(a) * pr);
        glVertex3f(x + cosf(a) * pr * 0.5f, ph, z + sinf(a) * pr * 0.5f);
    }
    glEnd();
}

void drawTreeShadow(float x, float z, float h) {
    float cr = h * 0.38f, th = h * 0.38f;
    glBegin(GL_TRIANGLE_STRIP);
    for (int i = 0; i <= 8; i++) { float a = (float)i / 8 * 2 * PI; glVertex3f(x + cosf(a) * 0.25f, 0.05f, z + sinf(a) * 0.25f); glVertex3f(x + cosf(a) * 0.15f, th, z + sinf(a) * 0.15f); }
    glEnd();
    glBegin(GL_TRIANGLE_FAN);
    glVertex3f(x, th + h * 0.28f, z);
    for (int i = 0; i <= 10; i++) { float a = (float)i / 10 * 2 * PI; glVertex3f(x + cosf(a) * cr, th, z + sinf(a) * cr); }
    glEnd();
}

// DESENEAZA TOATE OBIECTELE 
void drawAllObjects() {
    glEnable(GL_LIGHTING);

    float treeData[][3] = {
        {-5,5,0},{5,5,1},{0,-5,0.5f},{-8,-8,0.3f},{8,-5,0.7f},{3,10,0},{-5,15,0.4f},{12,8,0.6f},
        {-70,-20,0},{-75,0,0.5f},{-72,20,0.3f},{-68,40,0.7f},{-80,-40,0.2f},{-65,-55,0.8f},{-78,55,0.1f},
        {70,-20,0.4f},{75,0,0},{72,20,0.6f},{68,40,0.2f},{80,-40,0.9f},{65,-55,0.3f},{78,55,0.7f},
        {-30,70,0},{0,75,0.5f},{30,70,0.3f},{-20,-70,0.8f},{0,-72,0.1f},{25,-68,0.6f},{-50,65,0.4f},{50,65,0.2f},
        {-40,-60,0},{-44,-58,0.3f},{-42,-63,0.7f},{40,60,0.1f},{44,58,0.5f},{42,63,0.9f},
    };
    int numTrees = sizeof(treeData) / sizeof(treeData[0]);
    for (int i = 0; i < numTrees; i++) {
        float h = 3.5f + treeData[i][2] * 2.5f;
        drawTree(treeData[i][0], treeData[i][1], h, treeData[i][2]);
    }

    for (auto& p : polePositions) {
        float rot = atan2f(p.x, p.z) * 180.0f / PI;
        drawLightPoleAt(p.x, p.z, rot);
    }

    drawBuilding(-75, -15, 12, 8, 10, 0.82f, 0.82f, 0.85f);
    drawBuilding(-75, 10, 10, 7, 8, 0.75f, 0.78f, 0.82f);
    drawBuilding(62, -15, 10, 8, 9, 0.80f, 0.76f, 0.76f);
    drawBuilding(62, 10, 8, 7, 12, 0.70f, 0.72f, 0.80f);
    drawBuilding(-20, -75, 14, 6, 7, 0.78f, 0.75f, 0.70f);
    drawBuilding(10, -75, 12, 6, 9, 0.82f, 0.80f, 0.75f);

    drawTribune(55, -8, 18, 6, 90);
    drawTribune(-60, -5, 16, 5, -90);
    drawTribune(-15, 58, 20, 4, 180);
    drawTribune(-10, -62, 18, 4, 0);

    // Conuri - deseneaza din knockableCones (cu animatie cadere)
    for (auto& c : knockableCones) {
        glPushMatrix();
        glTranslatef(c.curX, 0, c.curZ);
        if (c.knocked) {
            glRotatef(c.fallAngle, c.fallAxis[1], 0, -c.fallAxis[0]);
        }
        glTranslatef(-c.curX, 0, -c.curZ);
        drawCone(c.curX, c.curZ);
        glPopMatrix();
    }

    float billData[][3] = { {58,-5,90},{58,5,90},{-58,15,0},{-60,0,0},{-58,-15,0},{-5,-58,90},{15,-58,90},{35,-58,90},{45,50,45},{25,58,0} };
    int numBills = sizeof(billData) / sizeof(billData[0]);
    for (int i = 0; i < numBills; i++) drawBillboard(billData[i][0], billData[i][1], billData[i][2], i);
}

//DESENEAZA UMBRELE PLATE 
void drawShadows() {
    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(0.0f, 0.0f, 0.0f, 0.35f);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(-2.0f, -2.0f);

    glPushMatrix();
    setShadowMatrix(sunDir[0], sunDir[1], sunDir[2], sunDir[3]);

    float treeData[][3] = {
        {-5,5,4},{5,5,5},{0,-5,4.5f},{-8,-8,3.8f},{8,-5,4.7f},{3,10,4},{-5,15,4.4f},{12,8,4.6f},
        {-70,-20,4},{-75,0,4.5f},{-72,20,4.3f},{70,-20,4.4f},{75,0,4},{72,20,4.6f},
    };
    for (int i = 0; i < 14; i++) drawTreeShadow(treeData[i][0], treeData[i][1], treeData[i][2]);

    for (auto& p : polePositions) drawPoleShadow(p.x, p.z);

    // ===== C1: Umbra masina =====
    playerCar.drawShadow();

    glPopMatrix();

    if (nightMode && polePositions.size() > 0) {
        int maxShadowLights = 4;
        int step = fmaxf(1, (int)polePositions.size() / maxShadowLights);
        for (int li = 0; li < (int)polePositions.size() && li / step < maxShadowLights; li += step) {
            auto& lp = polePositions[li];
            float lh = 6.2f;
            glPushMatrix();
            setShadowMatrix(lp.x, lh, lp.z, 1.0f);
            for (int i = 0; i < 14; i++) {
                float dx = treeData[i][0] - lp.x, dz = treeData[i][1] - lp.z;
                if (sqrtf(dx * dx + dz * dz) < 30.0f)
                    drawTreeShadow(treeData[i][0], treeData[i][1], treeData[i][2]);
            }
            glPopMatrix();
        }
    }

    glDisable(GL_POLYGON_OFFSET_FILL);
    glDisable(GL_BLEND);
    glEnable(GL_LIGHTING);
}

//CONFIGUREAZA LUMINILE 
void setupLights() {
    GLfloat sunAmb[] = { 0.30f,0.28f,0.25f,1.0f };
    GLfloat sunDiff[] = { 1.00f,0.95f,0.80f,1.0f };
    GLfloat sunSpec[] = { 0.50f,0.50f,0.40f,1.0f };
    if (nightMode) {
        sunAmb[0] = 0.05f; sunAmb[1] = 0.05f; sunAmb[2] = 0.10f;
        sunDiff[0] = 0.15f; sunDiff[1] = 0.15f; sunDiff[2] = 0.25f;
        sunSpec[0] = 0.10f; sunSpec[1] = 0.10f; sunSpec[2] = 0.15f;
    }
    glLightfv(GL_LIGHT0, GL_POSITION, sunDir);
    glLightfv(GL_LIGHT0, GL_AMBIENT, sunAmb);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, sunDiff);
    glLightfv(GL_LIGHT0, GL_SPECULAR, sunSpec);
    glEnable(GL_LIGHT0);

    for (int i = 1; i < MAX_LIGHTS; i++) glDisable(GL_LIGHT0 + i);

    if (nightMode && polePositions.size() > 0) {
        int maxStreetLights = 7;
        int step = fmaxf(1, (int)polePositions.size() / maxStreetLights);
        int lightIdx = 1;
        for (int i = 0; i < (int)polePositions.size() && lightIdx < MAX_LIGHTS; i += step) {
            auto& lp = polePositions[i];
            GLenum lightID = GL_LIGHT0 + lightIdx;
            GLfloat pos[] = { lp.x,6.2f,lp.z,1.0f };
            GLfloat amb[] = { 0.02f,0.02f,0.01f,1.0f };
            GLfloat diff[] = { 1.0f,0.90f,0.55f,1.0f };
            GLfloat spec[] = { 0.5f,0.45f,0.2f,1.0f };
            glLightfv(lightID, GL_POSITION, pos);
            glLightfv(lightID, GL_AMBIENT, amb);
            glLightfv(lightID, GL_DIFFUSE, diff);
            glLightfv(lightID, GL_SPECULAR, spec);
            glLightf(lightID, GL_CONSTANT_ATTENUATION, 1.0f);
            glLightf(lightID, GL_LINEAR_ATTENUATION, 0.05f);
            glLightf(lightID, GL_QUADRATIC_ATTENUATION, 0.008f);
            glEnable(lightID);
            lightIdx++;
        }
    }
}

// APLICA CAMERA
void applyCamera() {
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // ===== C1: Camera follow masina =====
    if (cameraFollow) {
        float rad = playerCar.angle * PI / 180.0f;
        float behindDist = 12.0f;
        float heightAbove = 5.0f;
        float eyeX = playerCar.x - sinf(rad) * behindDist;
        float eyeY = playerCar.y + heightAbove;
        float eyeZ = playerCar.z - cosf(rad) * behindDist;
        float lookX = playerCar.x + sinf(rad) * 5.0f;
        float lookY = playerCar.y + 1.0f;
        float lookZ = playerCar.z + cosf(rad) * 5.0f;
        gluLookAt(eyeX, eyeY, eyeZ, lookX, lookY, lookZ, 0, 1, 0);
        return;
    }

    if (firstPerson) {
        glRotatef(-camPitch, 1, 0, 0);
        glRotatef(-camYaw, 0, 1, 0);
        glTranslatef(-camX, -camY, -camZ);
    }
    else {
        float rad = camYaw * PI / 180.0f;
        float pitch = camPitch * PI / 180.0f;
        float dx = -sinf(rad) * cosf(pitch) * camDist;
        float dy = sinf(pitch) * camDist;
        float dz = -cosf(rad) * cosf(pitch) * camDist;
        glRotatef(-camPitch, 1, 0, 0);
        glRotatef(-camYaw, 0, 1, 0);
        glTranslatef(-(camX + dx), -(camY + dy), -(camZ + dz));
    }
}

// ===== C1+C2: UPDATE LOGICA =====
void updateGame(float dt) {
    // Win timer
    if (raceWon) {
        winTimer -= dt;
        if (winTimer <= 0) winTimer = 0;
        return;  // opreste jocul dupa victorie
    }

    // Update masina
    playerCar.update(dt, keyArrowUp, keyArrowDown, keyArrowLeft, keyArrowRight,
        trackSpline, ROAD_WIDTH);

    // === LAP DETECTION ===
    // Linia de start e la trackSpline[0], perpendiculara pe directia drumului
    Vec2 startPt = trackSpline[0];
    Vec2 nextPt = trackSpline[1];
    // Directia drumului la start
    float dirX = nextPt.x - startPt.x;
    float dirZ = nextPt.z - startPt.z;
    float dirLen = sqrtf(dirX * dirX + dirZ * dirZ);
    if (dirLen > 0.001f) { dirX /= dirLen; dirZ /= dirLen; }
    // Vector de la start la masina (proiectat pe directia drumului)
    float toCarX = playerCar.x - startPt.x;
    float toCarZ = playerCar.z - startPt.z;
    float dotAlong = toCarX * dirX + toCarZ * dirZ;  // pozitiv = dupa linie
    // Distanta laterala de la linia de start
    float crossDist = fabsf(toCarX * (-dirZ) + toCarZ * dirX);

    if (crossDist < ROAD_WIDTH + 2.0f) {  // aproape de drum
        if (dotAlong > 1.0f && dotAlong < 8.0f && !crossedStart) {
            // Tocmai a trecut linia (e putin dupa ea, in directia corecta)
            crossedStart = true;
            lapCount++;
            printf("=== LAP %d / %d ===\n", lapCount, lapTarget);
            if (lapCount >= lapTarget) {
                raceWon = true;
                winTimer = 10.0f;
                printf("!!! FELICITARI - AI CASTIGAT CURSA !!!\n");
            }
        }
        if (dotAlong < -5.0f) {
            // S-a indepartat destul inapoi - permite urmatoarea detectie
            crossedStart = false;
        }
    }

    // Coliziuni masina-cladiri si tribune
    for (auto& box : buildingBoxes) {
        if (checkCarAABB(playerCar, box)) {
            resolveCarBuilding(playerCar, box);
        }
    }

    // Coliziuni masina-conuri (knockable - cad, nu blocheaza)
    for (auto& cone : knockableCones) {
        if (checkCarCone(playerCar, cone)) {
            knockCone(playerCar, cone);
        }
    }
    updateCones(knockableCones, dt);

    // Coliziuni masina-stalpi (solide)
    for (auto& pole : poleColliders) {
        if (checkCarCircle(playerCar, pole.x, pole.z, pole.radius)) {
            resolveCarCircle(playerCar, pole.x, pole.z, pole.radius);
        }
    }

    // C2: Update ambulante
    int splineSize = (int)trackSpline.size();
    for (int i = 0; i < NUM_AMBULANCES; i++) {
        ambulances[i].update(dt, trackSpline, splineSize);

        // Coliziune masina-ambulanta
        if (checkCarAmbulance(playerCar, ambulances[i])) {
            resolveCarAmbulance(playerCar, ambulances[i]);
        }
    }

    // Update pietoni
    for (int i = 0; i < NUM_PEDESTRIANS; i++) {
        pedestrians[i].update(dt, trackSpline, ROAD_WIDTH);

        // Coliziune masina-pieton
        if (!pedestrians[i].hit && pedestrians[i].isCrossing) {
            if (checkCarPedestrian(playerCar, pedestrians[i])) {
                pedestrians[i].onHit();
                playerCar.speed *= 0.3f;
                printf("ATENTIE: Pieton lovit!\n");
            }
        }
    }
}

//DISPLAY 
void display() {
    if (nightMode) glClearColor(0.02f, 0.03f, 0.08f, 1.0f);
    else          glClearColor(0.50f, 0.75f, 1.00f, 1.0f);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

    // ===== C1: Update logica =====
    float dt = getDeltaTime();
    updateGame(dt);

    applyCamera();
    setupLights();

    glPushMatrix();
    if (cameraFollow) {
        // Skybox centrat pe camera follow
        float rad = playerCar.angle * PI / 180.0f;
        float eyeX = playerCar.x - sinf(rad) * 12.0f;
        float eyeY = playerCar.y + 5.0f;
        float eyeZ = playerCar.z - cosf(rad) * 12.0f;
        glTranslatef(eyeX, eyeY, eyeZ);
    }
    else {
        glTranslatef(camX, camY, camZ);
    }
    drawSkybox();
    glPopMatrix();

    drawTerrain();
    drawCircuit();
    drawBarriers();
    drawStartLine();
    drawShadows();
    drawAllObjects();

    // ===== C1+C2: Deseneaza masina, pietoni, ambulante =====
    playerCar.draw();
    for (int i = 0; i < NUM_PEDESTRIANS; i++) {
        pedestrians[i].draw();
    }
    for (int i = 0; i < NUM_AMBULANCES; i++) {
        ambulances[i].draw();
    }

    // ===== HUD =====
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity();
    gluOrtho2D(0, winW, 0, winH);
    glMatrixMode(GL_MODELVIEW); glPushMatrix(); glLoadIdentity();

    // Viteza
    char speedText[64];
    sprintf(speedText, "Viteza: %.0f km/h", fabsf(playerCar.speed) * 3.6f);
    glColor3f(1, 1, 1);
    glRasterPos2f(20, winH - 30);
    for (char* c = speedText; *c; c++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c);

    // Lap counter
    char lapText[64];
    sprintf(lapText, "Lap: %d / %d", lapCount, lapTarget);
    glColor3f(1.0f, 1.0f, 0.2f);
    glRasterPos2f(20, winH - 55);
    for (char* c = lapText; *c; c++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c);

    // Mod camera
    const char* camText = cameraFollow ? "[C] Camera: FOLLOW" : "[C] Camera: FREE";
    glColor3f(0.8f, 0.8f, 0.8f);
    glRasterPos2f(20, winH - 80);
    for (const char* c = camText; *c; c++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, *c);

    // Mesaj victorie
    if (raceWon) {
        // Fundal semi-transparent
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glColor4f(0.0f, 0.0f, 0.0f, 0.6f);
        glBegin(GL_QUADS);
        glVertex2f(winW * 0.2f, winH * 0.35f);
        glVertex2f(winW * 0.8f, winH * 0.35f);
        glVertex2f(winW * 0.8f, winH * 0.65f);
        glVertex2f(winW * 0.2f, winH * 0.65f);
        glEnd();
        glDisable(GL_BLEND);

        // Text FELICITARI
        const char* winText = "FELICITARI! AI CASTIGAT CURSA!";
        glColor3f(1.0f, 1.0f, 0.0f);
        float textW = 0;
        for (const char* c = winText; *c; c++) textW += glutBitmapWidth(GLUT_BITMAP_TIMES_ROMAN_24, *c);
        glRasterPos2f((winW - textW) * 0.5f, winH * 0.52f);
        for (const char* c = winText; *c; c++) glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, *c);

        // Sub-text
        const char* subText = "Apasa R pentru a reporni cursa";
        glColor3f(1.0f, 1.0f, 1.0f);
        float subW = 0;
        for (const char* c = subText; *c; c++) subW += glutBitmapWidth(GLUT_BITMAP_HELVETICA_18, *c);
        glRasterPos2f((winW - subW) * 0.5f, winH * 0.44f);
        for (const char* c = subText; *c; c++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c);
    }

    glMatrixMode(GL_PROJECTION); glPopMatrix();
    glMatrixMode(GL_MODELVIEW); glPopMatrix();
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);

    glutSwapBuffers();
}

//INIT 
void init() {
    srand((unsigned)time(NULL));  // C1: seed random pt pietoni

    if (nightMode) glClearColor(0.02f, 0.03f, 0.08f, 1.0f);
    else          glClearColor(0.50f, 0.75f, 1.00f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);
    glShadeModel(GL_SMOOTH);

    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_LIGHTING);

    trackSpline = buildTrackSpline(20);
    printf("Circuit: %d puncte\n", (int)trackSpline.size());

    int n = (int)trackSpline.size();
    int poleStep = n / 16;
    for (int i = 0; i < n; i += poleStep) {
        Vec2 cur = trackSpline[i], nxt = trackSpline[(i + 1) % n];
        float tx = nxt.x - cur.x, tz = nxt.z - cur.z;
        float len = sqrtf(tx * tx + tz * tz); if (len < 0.001f)len = 0.001f;
        tx /= len; tz /= len;
        float nx2 = tz, nz2 = -tx;
        LightPos lp = { cur.x + nx2 * (ROAD_WIDTH + 2.5f),6.5f,cur.z + nz2 * (ROAD_WIDTH + 2.5f) };
        polePositions.push_back(lp);
    }
    printf("Stalpi: %d\n", (int)polePositions.size());

    generateTerrain();

    // ===== C1: Init masina =====
    playerCar.init();
    // Pozitioneaza masina pe linia de start
    playerCar.x = trackSpline[0].x;
    playerCar.z = trackSpline[0].z;
    // Calculeaza directia initiala
    Vec2 startDir = trackSpline[1];
    playerCar.angle = atan2f(startDir.x - trackSpline[0].x, startDir.z - trackSpline[0].z) * 180.0f / PI;
    playerCar.updateCorners();

    // ===== C1: Init pietoni =====
    for (int i = 0; i < NUM_PEDESTRIANS; i++) {
        pedestrians[i].init(trackSpline, ROAD_WIDTH);
    }

    // ===== C1+C2: Init coliziuni complete =====
    buildingBoxes = getBuildingAABBs();
    knockableCones = getKnockableCones();
    poleColliders = getPoleColliders(polePositions);
    printf("Collidere AABB: %d (cladiri+tribune)\n", (int)buildingBoxes.size());
    printf("Conuri knockable: %d, Stalpi: %d\n", (int)knockableCones.size(), (int)poleColliders.size());

    // ===== C2: Init ambulante =====
    // Ambulanta 1: pe exteriorul drumului, viteza moderata
    ambulances[0].init(ROAD_WIDTH + 3.5f, 8.0f);
    ambulances[0].splinePos = 0.0f;
    // Ambulanta 2: pe interiorul drumului, directia opusa (viteza negativa)
    ambulances[1].init(-(ROAD_WIDTH + 3.5f), 6.0f);
    ambulances[1].splinePos = (float)(trackSpline.size() / 2);
    printf("Ambulante: %d\n", NUM_AMBULANCES);

    // Init timer
    lastTime = glutGet(GLUT_ELAPSED_TIME) / 1000.0f;

#define PATH "C:\\Users\\Darius\\Documents\\FACULTATE_ACE_LICENTA\\FACULTATE_ANUL_IV_SEM_II\\SPG\\SPG_PROIECT\\textures\\"
    texGrass = loadTexture(PATH"grass.jpg", 80, 160, 60);
    texSkyFront = loadTexture(PATH"sky_front.jpg", 100, 160, 220);
    texSkyBack = loadTexture(PATH"sky_back.jpg", 100, 160, 220);
    texSkyLeft = loadTexture(PATH"sky_left.jpg", 100, 160, 220);
    texSkyRight = loadTexture(PATH"sky_right.jpg", 100, 160, 220);
    texSkyTop = loadTexture(PATH"sky_top.jpg", 80, 140, 220);
    texSkyBottom = loadTexture(PATH"sky_bottom.jpg", 100, 160, 220);
    texRoad = loadTexture(PATH"road.jpg", 60, 60, 60);
#undef PATH

    printf("\n=== SPG P1+P2+P3+C1+C2 ===\n");
    printf("  SAGETI          - controleaza masina\n");
    printf("  W/S/A/D + Q/E   - miscare camera (mod free)\n");
    printf("  Mouse stanga    - rotire camera\n");
    printf("  Mouse dreapta   - rotire camera alternativa\n");
    printf("  Scroll          - zoom in/out\n");
    printf("  C               - toggle camera FOLLOW / FREE\n");
    printf("  F               - toggle first/third person\n");
    printf("  N               - toggle zi / noapte\n");
    printf("  R               - reset camera + masina\n");
    printf("  ESC             - iesire\n\n");
}

//RESHAPE
void reshape(int w, int h) {
    winW = w; winH = h;
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION); glLoadIdentity();
    gluPerspective(65.0f, (float)w / h, 0.1f, 600.0f);
    glMatrixMode(GL_MODELVIEW);
}

//INPUT 
void mouseButton(int btn, int state, int x, int y) {
    if (btn == GLUT_LEFT_BUTTON) { leftDown = (state == GLUT_DOWN); lastMouseX = x; lastMouseY = y; }
    if (btn == GLUT_RIGHT_BUTTON) { rightDown = (state == GLUT_DOWN); lastMouseX = x; lastMouseY = y; }
    if (btn == 3) { camDist = fmaxf(1.0f, camDist - 1.5f); glutPostRedisplay(); }
    if (btn == 4) { camDist = fminf(80.0f, camDist + 1.5f); glutPostRedisplay(); }
}

void mouseMove(int x, int y) {
    if (leftDown || rightDown) {
        float sens = (leftDown) ? 0.3f : 0.2f;
        camYaw += (x - lastMouseX) * sens;
        camPitch += (y - lastMouseY) * sens;
        if (camPitch > 89)camPitch = 89;
        if (camPitch < -89)camPitch = -89;
        lastMouseX = x; lastMouseY = y;
        glutPostRedisplay();
    }
}

void keyboard(unsigned char key, int x, int y) {
    float spd = 0.8f, rad = camYaw * PI / 180.0f;
    switch (key) {
    case'w':case'W':camX -= sinf(rad) * spd; camZ -= cosf(rad) * spd; break;
    case's':case'S':camX += sinf(rad) * spd; camZ += cosf(rad) * spd; break;
    case'a':case'A':camX -= cosf(rad) * spd; camZ += sinf(rad) * spd; break;
    case'd':case'D':camX += cosf(rad) * spd; camZ -= sinf(rad) * spd; break;
    case'q':case'Q':camY += spd; break;
    case'e':case'E':camY -= spd; break;
    case'f':case'F':
        firstPerson = !firstPerson;
        printf("Camera: %s\n", firstPerson ? "First Person" : "Third Person");
        break;
    case'n':case'N':
        nightMode = !nightMode;
        printf("Mod: %s\n", nightMode ? "NOAPTE" : "ZI");
        break;
        // ===== C1: Camera follow =====
    case'c':case'C':
        cameraFollow = !cameraFollow;
        printf("Camera: %s\n", cameraFollow ? "FOLLOW CAR" : "FREE");
        break;
    case'r':case'R':
        camX = 0; camY = 8; camZ = 30; camYaw = 0; camPitch = -15; camDist = 1.0f;
        // Reset masina pe linia de start
        playerCar.x = trackSpline[0].x;
        playerCar.z = trackSpline[0].z;
        playerCar.speed = 0;
        playerCar.angle = atan2f(trackSpline[1].x - trackSpline[0].x,
            trackSpline[1].z - trackSpline[0].z) * 180.0f / PI;
        // Reset lap counter
        lapCount = 0;
        crossedStart = false;
        raceWon = false;
        winTimer = 0;
        printf("Reset complet - cursa reincepe!\n");
        break;
    case 27:exit(0);
    }
    float th = getHeight(camX, camZ);
    if (camY < th + 1.8f)camY = th + 1.8f;
    glutPostRedisplay();
}

// ===== C1: Taste speciale (sageti) =====
void specialKeyDown(int key, int x, int y) {
    switch (key) {
    case GLUT_KEY_UP:    keyArrowUp = true; break;
    case GLUT_KEY_DOWN:  keyArrowDown = true; break;
    case GLUT_KEY_LEFT:  keyArrowLeft = true; break;
    case GLUT_KEY_RIGHT: keyArrowRight = true; break;
    }
}

void specialKeyUp(int key, int x, int y) {
    switch (key) {
    case GLUT_KEY_UP:    keyArrowUp = false; break;
    case GLUT_KEY_DOWN:  keyArrowDown = false; break;
    case GLUT_KEY_LEFT:  keyArrowLeft = false; break;
    case GLUT_KEY_RIGHT: keyArrowRight = false; break;
    }
}

void idle() { glutPostRedisplay(); }

//MAIN 
int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_STENCIL);
    glutInitWindowSize(1280, 720);
    glutCreateWindow("SPG_PROIECT - P1+P2+P3+C1+C2");
    init();
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutMouseFunc(mouseButton);
    glutMotionFunc(mouseMove);
    // ===== C1: Sageti =====
    glutSpecialFunc(specialKeyDown);
    glutSpecialUpFunc(specialKeyUp);
    glutIdleFunc(idle);
    glutMainLoop();
    return 0;
}