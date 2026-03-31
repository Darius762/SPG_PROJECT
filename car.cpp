#include "car.h"
#include <freeglut.h>
#include <stdio.h>

// Forward declaration
struct Vec2 { float x, z; };

void Car::init() {
    x = 50.0f;  y = 0.0f;  z = 0.0f;
    angle = 90.0f;
    speed = 0.0f;
    maxSpeed = 35.0f;
    acceleration = 12.0f;
    brakeForce = 18.0f;
    friction = 5.0f;
    turnSpeed = 90.0f;
    wheelAngle = 0.0f;
    wheelSpin = 0.0f;
    width = 1.8f;
    length = 3.8f;
    height = 1.0f;
}

bool Car::isOnRoad(float px, float pz, const std::vector<Vec2>& trackSpline, float roadWidth) {
    float minD = 9999.0f;
    for (auto& p : trackSpline) {
        float dx = px - p.x, dz = pz - p.z;
        float d = sqrtf(dx * dx + dz * dz);
        if (d < minD) minD = d;
    }
    return minD < roadWidth + 1.0f;
}

void Car::updateCorners() {
    float rad = angle * PI / 180.0f;
    float cosA = cosf(rad), sinA = sinf(rad);
    float hw = width * 0.5f, hl = length * 0.5f;
    // 4 colturi relative, rotite
    float lx[4] = { -hw, hw, hw, -hw };
    float lz[4] = { -hl, -hl, hl, hl };
    for (int i = 0; i < 4; i++) {
        corners[i][0] = x + lx[i] * cosA - lz[i] * sinA;
        corners[i][1] = z + lx[i] * sinA + lz[i] * cosA;
    }
}

void Car::update(float dt, bool keyUp, bool keyDown, bool keyLeft, bool keyRight,
    const std::vector<Vec2>& trackSpline, float roadWidth) {
    // Accelerare / franare
    if (keyUp) {
        speed += acceleration * dt;
        if (speed > maxSpeed) speed = maxSpeed;
    }
    else if (keyDown) {
        speed -= brakeForce * dt;
        if (speed < -maxSpeed * 0.3f) speed = -maxSpeed * 0.3f;
    }
    else {
        // Frecare
        if (speed > 0) {
            speed -= friction * dt;
            if (speed < 0) speed = 0;
        }
        else if (speed < 0) {
            speed += friction * dt;
            if (speed > 0) speed = 0;
        }
    }

    // Virare (doar cand se misca)
    float turnFactor = fminf(1.0f, fabsf(speed) / 5.0f);
    if (keyLeft) {
        angle += turnSpeed * dt * turnFactor * (speed >= 0 ? 1.0f : -1.0f);
        wheelAngle = fminf(wheelAngle + 120.0f * dt, 30.0f);
    }
    else if (keyRight) {
        angle -= turnSpeed * dt * turnFactor * (speed >= 0 ? 1.0f : -1.0f);
        wheelAngle = fmaxf(wheelAngle - 120.0f * dt, -30.0f);
    }
    else {
        // Rotile revin la centru
        if (wheelAngle > 0) { wheelAngle -= 150.0f * dt; if (wheelAngle < 0) wheelAngle = 0; }
        if (wheelAngle < 0) { wheelAngle += 150.0f * dt; if (wheelAngle > 0) wheelAngle = 0; }
    }

    // Miscare
    float rad = angle * PI / 180.0f;
    float newX = x + sinf(rad) * speed * dt;
    float newZ = z + cosf(rad) * speed * dt;

    // Verificare daca ramane pe drum (sau aproape)
    if (isOnRoad(newX, newZ, trackSpline, roadWidth)) {
        x = newX;
        z = newZ;
    }
    else {
        // Incetineste la iesirea de pe drum
        speed *= 0.85f;
        x = newX;
        z = newZ;
    }

    // Rotatie roti (animatie)
    wheelSpin += speed * dt * 60.0f;
    if (wheelSpin > 360.0f) wheelSpin -= 360.0f;
    if (wheelSpin < -360.0f) wheelSpin += 360.0f;

    updateCorners();
}

// ---- DESENARE MASINA DETALIATA ----

static void drawWheel(float radius, float width) {
    int seg = 12;
    float hw = width * 0.5f;
    // Cauciuc
    glColor3f(0.15f, 0.15f, 0.15f);
    glBegin(GL_TRIANGLE_STRIP);
    for (int i = 0; i <= seg; i++) {
        float a = (float)i / seg * 2.0f * PI;
        float cs = cosf(a), sn = sinf(a);
        glNormal3f(cs, sn, 0);
        glVertex3f(cs * radius, sn * radius, -hw);
        glVertex3f(cs * radius, sn * radius, hw);
    }
    glEnd();
    // Janta
    glColor3f(0.7f, 0.7f, 0.75f);
    for (int side = -1; side <= 1; side += 2) {
        glBegin(GL_TRIANGLE_FAN);
        glNormal3f(0, 0, (float)side);
        glVertex3f(0, 0, side * hw);
        for (int i = 0; i <= seg; i++) {
            float a = (float)i / seg * 2.0f * PI;
            glVertex3f(cosf(a) * radius * 0.7f, sinf(a) * radius * 0.7f, side * hw);
        }
        glEnd();
    }
    // Spite
    glColor3f(0.8f, 0.8f, 0.85f);
    for (int i = 0; i < 5; i++) {
        float a = (float)i / 5 * 2.0f * PI;
        glBegin(GL_LINES);
        glVertex3f(0, 0, hw + 0.01f);
        glVertex3f(cosf(a) * radius * 0.65f, sinf(a) * radius * 0.65f, hw + 0.01f);
        glEnd();
    }
}

void Car::draw() {
    glDisable(GL_TEXTURE_2D);
    glEnable(GL_LIGHTING);

    glPushMatrix();
    glTranslatef(x, y, z);
    glRotatef(angle, 0, 1, 0);

    float hw = width * 0.5f, hl = length * 0.5f;
    float bh = 0.35f;  // inaltimea bazei de la sol
    float wR = 0.28f;   // raza roata

    // --- SASIU (platforma de baza) ---
    glColor3f(0.25f, 0.25f, 0.28f);
    glPushMatrix();
    glTranslatef(0, bh - 0.05f, 0);
    glScalef(width * 0.95f, 0.1f, length * 0.9f);
    glutSolidCube(1.0f);
    glPopMatrix();

    // --- CAROSERIE PRINCIPALA (corp jos) ---
    glColor3f(0.85f, 0.12f, 0.12f);  // rosu racing
    // Partea de jos a caroseriei
    glBegin(GL_QUADS);
    // Laterala stanga
    glNormal3f(-1, 0, 0);
    glVertex3f(-hw, bh, -hl);  glVertex3f(-hw, bh, hl * 0.85f);
    glVertex3f(-hw, bh + height * 0.45f, hl * 0.85f);  glVertex3f(-hw, bh + height * 0.45f, -hl);
    // Laterala dreapta
    glNormal3f(1, 0, 0);
    glVertex3f(hw, bh, -hl);  glVertex3f(hw, bh + height * 0.45f, -hl);
    glVertex3f(hw, bh + height * 0.45f, hl * 0.85f);  glVertex3f(hw, bh, hl * 0.85f);
    // Capota (fata sus)
    glNormal3f(0, 1, 0);
    glVertex3f(-hw, bh + height * 0.45f, hl * 0.3f);  glVertex3f(hw, bh + height * 0.45f, hl * 0.3f);
    glVertex3f(hw, bh + height * 0.45f, hl * 0.85f);  glVertex3f(-hw, bh + height * 0.45f, hl * 0.85f);
    // Fata
    glNormal3f(0, 0, 1);
    glVertex3f(-hw, bh, hl * 0.85f);  glVertex3f(hw, bh, hl * 0.85f);
    glVertex3f(hw, bh + height * 0.35f, hl * 0.85f);  glVertex3f(-hw, bh + height * 0.35f, hl * 0.85f);
    // Spate
    glNormal3f(0, 0, -1);
    glVertex3f(-hw, bh, -hl);  glVertex3f(-hw, bh + height * 0.5f, -hl);
    glVertex3f(hw, bh + height * 0.5f, -hl);  glVertex3f(hw, bh, -hl);
    glEnd();

    // --- CABINA (partea de sus) ---
    float cabFront = hl * 0.3f, cabBack = -hl * 0.45f;
    float cabH = bh + height;
    float cabBot = bh + height * 0.45f;
    // Geamuri - albastru inchis
    glColor3f(0.15f, 0.2f, 0.35f);
    glBegin(GL_QUADS);
    // Parbriz (inclinat)
    glNormal3f(0, 0.3f, 1);
    glVertex3f(-hw + 0.1f, cabBot, cabFront);  glVertex3f(hw - 0.1f, cabBot, cabFront);
    glVertex3f(hw * 0.7f, cabH, cabFront * 0.6f);  glVertex3f(-hw * 0.7f, cabH, cabFront * 0.6f);
    // Luneta (inclinata)
    glNormal3f(0, 0.3f, -1);
    glVertex3f(-hw + 0.1f, cabBot, cabBack);  glVertex3f(-hw * 0.7f, cabH, cabBack * 0.7f);
    glVertex3f(hw * 0.7f, cabH, cabBack * 0.7f);  glVertex3f(hw - 0.1f, cabBot, cabBack);
    // Geam stanga
    glNormal3f(-1, 0.2f, 0);
    glVertex3f(-hw + 0.05f, cabBot, cabFront);  glVertex3f(-hw * 0.7f, cabH, cabFront * 0.6f);
    glVertex3f(-hw * 0.7f, cabH, cabBack * 0.7f);  glVertex3f(-hw + 0.05f, cabBot, cabBack);
    // Geam dreapta
    glNormal3f(1, 0.2f, 0);
    glVertex3f(hw - 0.05f, cabBot, cabFront);  glVertex3f(hw - 0.05f, cabBot, cabBack);
    glVertex3f(hw * 0.7f, cabH, cabBack * 0.7f);  glVertex3f(hw * 0.7f, cabH, cabFront * 0.6f);
    glEnd();

    // Acoperis cabina
    glColor3f(0.80f, 0.10f, 0.10f);
    glBegin(GL_QUADS);
    glNormal3f(0, 1, 0);
    glVertex3f(-hw * 0.7f, cabH, cabFront * 0.6f);  glVertex3f(hw * 0.7f, cabH, cabFront * 0.6f);
    glVertex3f(hw * 0.7f, cabH, cabBack * 0.7f);  glVertex3f(-hw * 0.7f, cabH, cabBack * 0.7f);
    glEnd();

    // --- SPOILER ---
    glColor3f(0.2f, 0.2f, 0.22f);
    // Stalpi spoiler
    float spY = bh + height * 0.5f, spTop = spY + 0.5f;
    for (int s = -1; s <= 1; s += 2) {
        glBegin(GL_QUADS);
        glVertex3f(s * hw * 0.6f, spY, -hl + 0.1f);  glVertex3f(s * hw * 0.6f + s * 0.05f, spY, -hl + 0.1f);
        glVertex3f(s * hw * 0.6f + s * 0.05f, spTop, -hl + 0.05f);  glVertex3f(s * hw * 0.6f, spTop, -hl + 0.05f);
        glEnd();
    }
    // Aripa spoiler
    glColor3f(0.15f, 0.15f, 0.18f);
    glBegin(GL_QUADS);
    glNormal3f(0, 1, 0);
    glVertex3f(-hw * 0.75f, spTop, -hl);  glVertex3f(hw * 0.75f, spTop, -hl);
    glVertex3f(hw * 0.75f, spTop, -hl + 0.4f);  glVertex3f(-hw * 0.75f, spTop, -hl + 0.4f);
    glNormal3f(0, -1, 0);
    glVertex3f(-hw * 0.75f, spTop - 0.06f, -hl);  glVertex3f(-hw * 0.75f, spTop - 0.06f, -hl + 0.4f);
    glVertex3f(hw * 0.75f, spTop - 0.06f, -hl + 0.4f);  glVertex3f(hw * 0.75f, spTop - 0.06f, -hl);
    glEnd();

    // --- FARURI FATA ---
    glColor3f(1.0f, 1.0f, 0.85f);
    for (int s = -1; s <= 1; s += 2) {
        glPushMatrix();
        glTranslatef(s * hw * 0.65f, bh + height * 0.22f, hl * 0.86f);
        glutSolidSphere(0.12f, 8, 6);
        glPopMatrix();
    }

    // --- STOPURI SPATE ---
    glColor3f(1.0f, 0.0f, 0.0f);
    for (int s = -1; s <= 1; s += 2) {
        glPushMatrix();
        glTranslatef(s * hw * 0.7f, bh + height * 0.3f, -hl + 0.02f);
        glScalef(0.2f, 0.12f, 0.05f);
        glutSolidCube(1.0f);
        glPopMatrix();
    }

    // --- ROTI ---
    float wheelZ_front = hl * 0.6f;
    float wheelZ_back = -hl * 0.55f;
    float wheelX = hw + 0.05f;
    float wheelW = 0.2f;

    // Roti fata (se rotesc cu directia)
    for (int s = -1; s <= 1; s += 2) {
        glPushMatrix();
        glTranslatef(s * wheelX, wR + 0.05f, wheelZ_front);
        glRotatef(wheelAngle, 0, 1, 0);  // directie
        glRotatef(90, 0, 1, 0);          // orientare laterala
        glRotatef(-wheelSpin, 0, 0, 1);  // rotatie
        drawWheel(wR, wheelW);
        glPopMatrix();
    }
    // Roti spate
    for (int s = -1; s <= 1; s += 2) {
        glPushMatrix();
        glTranslatef(s * wheelX, wR + 0.05f, wheelZ_back);
        glRotatef(90, 0, 1, 0);          // orientare laterala
        glRotatef(-wheelSpin, 0, 0, 1);
        drawWheel(wR, wheelW);
        glPopMatrix();
    }

    glPopMatrix();
}

void Car::drawShadow() {
    glPushMatrix();
    glTranslatef(x, 0.06f, z);
    glRotatef(angle, 0, 1, 0);

    float hw = width * 0.55f, hl = length * 0.55f;
    glBegin(GL_QUADS);
    glVertex3f(-hw, 0, -hl);  glVertex3f(hw, 0, -hl);
    glVertex3f(hw, 0, hl);    glVertex3f(-hw, 0, hl);
    glEnd();

    glPopMatrix();
}