#ifndef AMBULANCE_H
#define AMBULANCE_H

#include <vector>
#include <math.h>

#ifndef PI
#define PI 3.14159265f
#endif

struct Vec2;

struct Ambulance {
    float x, y, z;
    float angle;
    float speed;
    float splinePos;       // pozitie pe spline (float index)
    float sideOffset;      // offset lateral fata de drum
    float sirenPhase;      // animatie sirena
    float wheelSpin;
    float width, length, height;

    // Bounding box corners
    float corners[4][2];

    void init(float offset, float spd);
    void update(float dt, const std::vector<Vec2>& trackSpline, int splineSize);
    void draw();
    void drawShadow();
    void updateCorners();
};

#endif