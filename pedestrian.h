#ifndef PEDESTRIAN_H
#define PEDESTRIAN_H

#include <vector>

#ifndef PI
#define PI 3.14159265f
#endif

struct Vec2;

struct Pedestrian {
    float x, y, z;
    float targetX, targetZ;
    float startX, startZ;
    float speed;
    float angle;
    float walkCycle;       // animatie mers
    float waitTimer;       // timp asteptare inainte de traversare
    float crossProgress;   // 0..1 progres traversare
    bool  isCrossing;      // traverseaza?
    bool  isWaiting;       // asteapta sa traverseze?
    int   crossingPoint;   // indexul in spline unde traverseaza
    bool  crossDirection;  // directia traversarii
    bool  hit;             // a fost lovit?
    float hitTimer;        // timer dupa lovire

    void init(const std::vector<Vec2>& trackSpline, float roadWidth);
    void update(float dt, const std::vector<Vec2>& trackSpline, float roadWidth);
    void draw();
    void drawShadow();
    void startNewCrossing(const std::vector<Vec2>& trackSpline, float roadWidth);
    void onHit();          // cand e lovit de masina

    // Bounding box simplu
    float getRadius() { return 0.35f; }
};

#endif