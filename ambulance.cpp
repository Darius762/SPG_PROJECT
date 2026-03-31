#include "ambulance.h"
#include <freeglut.h>
#include <stdio.h>

struct Vec2 { float x, z; };

void Ambulance::init(float offset, float spd) {
    splinePos = 0.0f;
    sideOffset = offset;      // cat de departe de centrul drumului
    speed = spd;              // viteza pe spline
    sirenPhase = 0.0f;
    wheelSpin = 0.0f;
    width = 2.0f;
    length = 4.5f;
    height = 1.8f;
    x = y = z = 0;
    angle = 0;
}

void Ambulance::updateCorners() {
    float rad = angle * PI / 180.0f;
    float cosA = cosf(rad), sinA = sinf(rad);
    float hw = width * 0.5f, hl = length * 0.5f;
    float lx[4] = { -hw, hw, hw, -hw };
    float lz[4] = { -hl, -hl, hl, hl };
    for (int i = 0; i < 4; i++) {
        corners[i][0] = x + lx[i] * cosA - lz[i] * sinA;
        corners[i][1] = z + lx[i] * sinA + lz[i] * cosA;
    }
}

void Ambulance::update(float dt, const std::vector<Vec2>& trackSpline, int splineSize) {
    // Avanseaza pe spline
    splinePos += speed * dt;
    if (splinePos >= (float)splineSize) splinePos -= (float)splineSize;
    if (splinePos < 0) splinePos += (float)splineSize;

    int idx = (int)splinePos;
    float frac = splinePos - idx;
    int idx1 = idx % splineSize;
    int idx2 = (idx + 1) % splineSize;

    // Interpoleaza pozitie pe spline
    Vec2 p1 = trackSpline[idx1], p2 = trackSpline[idx2];
    float cx = p1.x + (p2.x - p1.x) * frac;
    float cz = p1.z + (p2.z - p1.z) * frac;

    // Calculeaza directia
    float tx = p2.x - p1.x, tz = p2.z - p1.z;
    float len = sqrtf(tx * tx + tz * tz);
    if (len < 0.001f) len = 0.001f;
    tx /= len; tz /= len;

    // Normala (perpendiculara pe drum)
    float nx = tz, nz = -tx;

    // Pozitionare pe lateralul drumului
    x = cx + nx * sideOffset;
    y = 0.0f;
    z = cz + nz * sideOffset;

    // Unghi in directia de mers
    angle = atan2f(tx, tz) * 180.0f / PI;

    // Animatii
    sirenPhase += dt * 6.0f;
    if (sirenPhase > 2.0f * PI) sirenPhase -= 2.0f * PI;

    wheelSpin += speed * dt * 15.0f;
    if (wheelSpin > 360.0f) wheelSpin -= 360.0f;

    updateCorners();
}

static void drawAmbWheel(float radius, float w) {
    int seg = 10;
    float hw = w * 0.5f;
    glColor3f(0.12f, 0.12f, 0.12f);
    glBegin(GL_TRIANGLE_STRIP);
    for (int i = 0; i <= seg; i++) {
        float a = (float)i / seg * 2.0f * PI;
        glVertex3f(cosf(a) * radius, sinf(a) * radius, -hw);
        glVertex3f(cosf(a) * radius, sinf(a) * radius, hw);
    }
    glEnd();
    glColor3f(0.6f, 0.6f, 0.65f);
    for (int side = -1; side <= 1; side += 2) {
        glBegin(GL_TRIANGLE_FAN);
        glVertex3f(0, 0, side * hw);
        for (int i = 0; i <= seg; i++) {
            float a = (float)i / seg * 2.0f * PI;
            glVertex3f(cosf(a) * radius * 0.6f, sinf(a) * radius * 0.6f, side * hw);
        }
        glEnd();
    }
}

void Ambulance::draw() {
    glDisable(GL_TEXTURE_2D);
    glEnable(GL_LIGHTING);

    glPushMatrix();
    glTranslatef(x, y, z);
    glRotatef(angle, 0, 1, 0);

    float hw = width * 0.5f, hl = length * 0.5f;
    float bh = 0.4f;
    float wR = 0.3f;

    // --- SASIU ---
    glColor3f(0.25f, 0.25f, 0.28f);
    glPushMatrix();
    glTranslatef(0, bh - 0.05f, 0);
    glScalef(width * 0.9f, 0.1f, length * 0.85f);
    glutSolidCube(1.0f);
    glPopMatrix();

    // --- CORP PRINCIPAL (alb) ---
    glColor3f(0.95f, 0.95f, 0.95f);
    glBegin(GL_QUADS);
    // Laterala stanga
    glNormal3f(-1, 0, 0);
    glVertex3f(-hw, bh, -hl); glVertex3f(-hw, bh, hl);
    glVertex3f(-hw, bh + height, hl); glVertex3f(-hw, bh + height, -hl);
    // Laterala dreapta
    glNormal3f(1, 0, 0);
    glVertex3f(hw, bh, -hl); glVertex3f(hw, bh + height, -hl);
    glVertex3f(hw, bh + height, hl); glVertex3f(hw, bh, hl);
    // Sus
    glNormal3f(0, 1, 0);
    glVertex3f(-hw, bh + height, -hl); glVertex3f(hw, bh + height, -hl);
    glVertex3f(hw, bh + height, hl); glVertex3f(-hw, bh + height, hl);
    // Fata
    glNormal3f(0, 0, 1);
    glVertex3f(-hw, bh, hl); glVertex3f(hw, bh, hl);
    glVertex3f(hw, bh + height * 0.7f, hl); glVertex3f(-hw, bh + height * 0.7f, hl);
    // Spate
    glNormal3f(0, 0, -1);
    glVertex3f(-hw, bh, -hl); glVertex3f(-hw, bh + height, -hl);
    glVertex3f(hw, bh + height, -hl); glVertex3f(hw, bh, -hl);
    glEnd();

    // --- PARBRIZ ---
    glColor3f(0.2f, 0.25f, 0.4f);
    glBegin(GL_QUADS);
    glNormal3f(0, 0.3f, 1);
    glVertex3f(-hw + 0.1f, bh + height * 0.35f, hl + 0.01f);
    glVertex3f(hw - 0.1f, bh + height * 0.35f, hl + 0.01f);
    glVertex3f(hw - 0.15f, bh + height * 0.7f, hl + 0.01f);
    glVertex3f(-hw + 0.15f, bh + height * 0.7f, hl + 0.01f);
    glEnd();

    // --- CRUCE ROSIE (pe laterale) ---
    glColor3f(0.9f, 0.1f, 0.1f);
    for (int s = -1; s <= 1; s += 2) {
        float sx = s * (hw + 0.01f);
        float cy = bh + height * 0.55f, cz = -hl * 0.3f;
        // Verticala crucii
        glBegin(GL_QUADS);
        glNormal3f((float)s, 0, 0);
        glVertex3f(sx, cy - 0.4f, cz - 0.12f); glVertex3f(sx, cy - 0.4f, cz + 0.12f);
        glVertex3f(sx, cy + 0.4f, cz + 0.12f); glVertex3f(sx, cy + 0.4f, cz - 0.12f);
        glEnd();
        // Orizontala crucii
        glBegin(GL_QUADS);
        glNormal3f((float)s, 0, 0);
        glVertex3f(sx, cy - 0.12f, cz - 0.4f); glVertex3f(sx, cy - 0.12f, cz + 0.4f);
        glVertex3f(sx, cy + 0.12f, cz + 0.4f); glVertex3f(sx, cy + 0.12f, cz - 0.4f);
        glEnd();
    }

    // --- BANDA PORTOCALIE ---
    glColor3f(1.0f, 0.55f, 0.0f);
    for (int s = -1; s <= 1; s += 2) {
        float sx = s * (hw + 0.01f);
        glBegin(GL_QUADS);
        glNormal3f((float)s, 0, 0);
        glVertex3f(sx, bh + height * 0.25f, -hl);
        glVertex3f(sx, bh + height * 0.25f, hl);
        glVertex3f(sx, bh + height * 0.35f, hl);
        glVertex3f(sx, bh + height * 0.35f, -hl);
        glEnd();
    }

    // --- SIRENA (pe acoperis) ---
    float sirenBlink = sinf(sirenPhase);
    // Sirena rosie (stanga)
    if (sirenBlink > 0) glColor3f(1.0f, 0.1f, 0.1f);
    else glColor3f(0.4f, 0.05f, 0.05f);
    glPushMatrix();
    glTranslatef(-0.3f, bh + height + 0.15f, 0);
    glutSolidSphere(0.15f, 8, 6);
    glPopMatrix();

    // Sirena albastra (dreapta)
    if (sirenBlink < 0) glColor3f(0.1f, 0.3f, 1.0f);
    else glColor3f(0.05f, 0.1f, 0.4f);
    glPushMatrix();
    glTranslatef(0.3f, bh + height + 0.15f, 0);
    glutSolidSphere(0.15f, 8, 6);
    glPopMatrix();

    // Baza sirenei
    glColor3f(0.3f, 0.3f, 0.35f);
    glPushMatrix();
    glTranslatef(0, bh + height + 0.05f, 0);
    glScalef(0.9f, 0.08f, 0.3f);
    glutSolidCube(1.0f);
    glPopMatrix();

    // --- FARURI ---
    glColor3f(1.0f, 1.0f, 0.8f);
    for (int s = -1; s <= 1; s += 2) {
        glPushMatrix();
        glTranslatef(s * hw * 0.6f, bh + height * 0.2f, hl + 0.01f);
        glutSolidSphere(0.1f, 6, 4);
        glPopMatrix();
    }

    // --- STOPURI ---
    glColor3f(1.0f, 0.0f, 0.0f);
    for (int s = -1; s <= 1; s += 2) {
        glPushMatrix();
        glTranslatef(s * hw * 0.6f, bh + height * 0.3f, -hl - 0.01f);
        glScalef(0.25f, 0.15f, 0.05f);
        glutSolidCube(1.0f);
        glPopMatrix();
    }

    // --- ROTI ---
    float wheelZ_front = hl * 0.55f;
    float wheelZ_back = -hl * 0.5f;
    float wheelX = hw + 0.05f;
    float wheelW = 0.22f;

    for (int s = -1; s <= 1; s += 2) {
        // Fata
        glPushMatrix();
        glTranslatef(s * wheelX, wR + 0.05f, wheelZ_front);
        glRotatef(90, 0, 1, 0);
        glRotatef(-wheelSpin, 0, 0, 1);
        drawAmbWheel(wR, wheelW);
        glPopMatrix();
        // Spate
        glPushMatrix();
        glTranslatef(s * wheelX, wR + 0.05f, wheelZ_back);
        glRotatef(90, 0, 1, 0);
        glRotatef(-wheelSpin, 0, 0, 1);
        drawAmbWheel(wR, wheelW);
        glPopMatrix();
    }

    glPopMatrix();
}

void Ambulance::drawShadow() {
    glPushMatrix();
    glTranslatef(x, 0.06f, z);
    glRotatef(angle, 0, 1, 0);
    float hw = width * 0.55f, hl = length * 0.55f;
    glBegin(GL_QUADS);
    glVertex3f(-hw, 0, -hl); glVertex3f(hw, 0, -hl);
    glVertex3f(hw, 0, hl);   glVertex3f(-hw, 0, hl);
    glEnd();
    glPopMatrix();
}