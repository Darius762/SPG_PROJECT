#include "pedestrian.h"
#include <freeglut.h>
#include <math.h>
#include <stdlib.h>

struct Vec2 { float x, z; };

void Pedestrian::init(const std::vector<Vec2>& trackSpline, float roadWidth) {
    speed = 2.5f;
    walkCycle = 0.0f;
    isCrossing = false;
    isWaiting = true;
    waitTimer = 2.0f + (rand() % 50) * 0.1f;  // 2-7 secunde prima asteptare
    hit = false;
    hitTimer = 0.0f;
    crossProgress = 0.0f;

    // Alege un punct initial pe marginea drumului
    startNewCrossing(trackSpline, roadWidth);
    // Pozitioneaza la start
    x = startX;
    y = 0.0f;
    z = startZ;
}

void Pedestrian::startNewCrossing(const std::vector<Vec2>& trackSpline, float roadWidth) {
    int n = (int)trackSpline.size();
    crossingPoint = rand() % n;

    Vec2 cur = trackSpline[crossingPoint];
    Vec2 nxt = trackSpline[(crossingPoint + 1) % n];
    float tx = nxt.x - cur.x, tz = nxt.z - cur.z;
    float len = sqrtf(tx * tx + tz * tz);
    if (len < 0.001f) len = 0.001f;
    tx /= len; tz /= len;

    float nx = tz, nz = -tx;
    float offset = roadWidth + 1.5f;

    crossDirection = (rand() % 2) == 0;

    if (crossDirection) {
        startX = cur.x + nx * offset;
        startZ = cur.z + nz * offset;
        targetX = cur.x - nx * offset;
        targetZ = cur.z - nz * offset;
    }
    else {
        startX = cur.x - nx * offset;
        startZ = cur.z - nz * offset;
        targetX = cur.x + nx * offset;
        targetZ = cur.z + nz * offset;
    }
}

void Pedestrian::update(float dt, const std::vector<Vec2>& trackSpline, float roadWidth) {
    // Daca a fost lovit, animatie de recuperare
    if (hit) {
        hitTimer -= dt;
        if (hitTimer <= 0) {
            hit = false;
            isCrossing = false;
            isWaiting = true;
            waitTimer = 3.0f + (rand() % 40) * 0.1f;
            startNewCrossing(trackSpline, roadWidth);
            x = startX;
            z = startZ;
        }
        return;
    }

    if (isWaiting) {
        waitTimer -= dt;
        if (waitTimer <= 0) {
            isWaiting = false;
            isCrossing = true;
            crossProgress = 0.0f;
            x = startX;
            z = startZ;
        }
        return;
    }

    if (isCrossing) {
        float dist = sqrtf((targetX - startX) * (targetX - startX) +
            (targetZ - startZ) * (targetZ - startZ));
        float travelTime = dist / speed;
        crossProgress += dt / travelTime;

        if (crossProgress >= 1.0f) {
            crossProgress = 1.0f;
            isCrossing = false;
            isWaiting = true;
            waitTimer = 3.0f + (rand() % 60) * 0.1f;  // 3-9 sec
            startNewCrossing(trackSpline, roadWidth);
        }

        x = startX + (targetX - startX) * crossProgress;
        z = startZ + (targetZ - startZ) * crossProgress;

        // Unghi catre tinta
        angle = atan2f(targetX - startX, targetZ - startZ) * 180.0f / PI;

        // Animatie mers
        walkCycle += dt * 8.0f;
        if (walkCycle > 2.0f * PI) walkCycle -= 2.0f * PI;
    }
}

void Pedestrian::draw() {
    glDisable(GL_TEXTURE_2D);
    glEnable(GL_LIGHTING);
    glPushMatrix();
    glTranslatef(x, y, z);
    glRotatef(angle, 0, 1, 0);

    if (hit) {
        // Cand e lovit, sta pe spate
        glRotatef(90, 1, 0, 0);
        glTranslatef(0, 0.5f, -0.3f);
    }

    float legSwing = isCrossing ? sinf(walkCycle) * 25.0f : 0.0f;
    float armSwing = isCrossing ? sinf(walkCycle) * 20.0f : 0.0f;

    // Picioare
    for (int s = -1; s <= 1; s += 2) {
        glPushMatrix();
        glTranslatef(s * 0.12f, 0.45f, 0);
        glRotatef(s * legSwing, 1, 0, 0);
        glTranslatef(0, -0.2f, 0);
        // Pantaloni - albastru
        glColor3f(0.15f, 0.2f, 0.45f);
        glPushMatrix();
        glScalef(0.14f, 0.45f, 0.14f);
        glutSolidCube(1.0f);
        glPopMatrix();
        // Pantof
        glColor3f(0.2f, 0.15f, 0.1f);
        glPushMatrix();
        glTranslatef(0, -0.25f, 0.04f);
        glScalef(0.15f, 0.08f, 0.2f);
        glutSolidCube(1.0f);
        glPopMatrix();
        glPopMatrix();
    }

    // Trunchi (tricou)
    glColor3f(0.9f, 0.85f, 0.2f);  // galben
    glPushMatrix();
    glTranslatef(0, 0.85f, 0);
    glScalef(0.35f, 0.4f, 0.2f);
    glutSolidCube(1.0f);
    glPopMatrix();

    // Brate
    for (int s = -1; s <= 1; s += 2) {
        glPushMatrix();
        glTranslatef(s * 0.24f, 0.95f, 0);
        glRotatef(-s * armSwing, 1, 0, 0);
        glTranslatef(0, -0.18f, 0);
        // Maneca
        glColor3f(0.9f, 0.85f, 0.2f);
        glPushMatrix();
        glTranslatef(0, 0.05f, 0);
        glScalef(0.1f, 0.15f, 0.1f);
        glutSolidCube(1.0f);
        glPopMatrix();
        // Mana (piele)
        glColor3f(0.85f, 0.7f, 0.55f);
        glPushMatrix();
        glTranslatef(0, -0.1f, 0);
        glScalef(0.08f, 0.15f, 0.08f);
        glutSolidCube(1.0f);
        glPopMatrix();
        glPopMatrix();
    }

    // Cap
    glColor3f(0.85f, 0.7f, 0.55f);
    glPushMatrix();
    glTranslatef(0, 1.25f, 0);
    glutSolidSphere(0.16f, 10, 8);
    glPopMatrix();

    // Par
    glColor3f(0.2f, 0.12f, 0.05f);
    glPushMatrix();
    glTranslatef(0, 1.32f, -0.02f);
    glScalef(0.17f, 0.1f, 0.17f);
    glutSolidCube(1.0f);
    glPopMatrix();

    glPopMatrix();
}

void Pedestrian::drawShadow() {
    if (hit) return;
    glPushMatrix();
    glTranslatef(x, 0.06f, z);
    glBegin(GL_TRIANGLE_FAN);
    glVertex3f(0, 0, 0);
    for (int i = 0; i <= 12; i++) {
        float a = (float)i / 12 * 2.0f * PI;
        glVertex3f(cosf(a) * 0.35f, 0, sinf(a) * 0.25f);
    }
    glEnd();
    glPopMatrix();
}

void Pedestrian::onHit() {
    if (!hit) {
        hit = true;
        hitTimer = 2.5f;  // sta jos 2.5 secunde
        isCrossing = false;
    }
}