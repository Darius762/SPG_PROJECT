#ifndef CAR_H
#define CAR_H

#include <vector>
#include <math.h>

#ifndef PI
#define PI 3.14159265f
#endif

struct Vec2;

struct Car {
    float x, y, z;          // pozitie
    float angle;             // directia (grade)
    float speed;             // viteza curenta
    float maxSpeed;          // viteza maxima
    float acceleration;      // acceleratie
    float brakeForce;        // forta franare
    float friction;          // frecare
    float turnSpeed;         // viteza virare
    float wheelAngle;        // unghi roti fata (vizual)
    float wheelSpin;         // rotatie roti (animatie)
    float width, length, height;  // dimensiuni

    // Bounding box corners (for collision)
    float corners[4][2];     // 4 colturi in world space

    void init();
    void update(float dt, bool keyUp, bool keyDown, bool keyLeft, bool keyRight,
        const std::vector<Vec2>& trackSpline, float roadWidth);
    void draw();
    void drawShadow();
    void updateCorners();

    // Verifica daca un punct e pe drum
    bool isOnRoad(float px, float pz, const std::vector<Vec2>& trackSpline, float roadWidth);
};

#endif