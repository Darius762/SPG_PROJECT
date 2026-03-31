#ifndef COLLISION_H
#define COLLISION_H

#include "car.h"
#include "pedestrian.h"
#include <vector>

struct Ambulance;  // forward declaration

struct LightPos { float x, y, z; };

struct AABB {
    float minX, minZ, maxX, maxZ;
};

struct CircleCollider {
    float x, z, radius;
};

// Con care poate fi lovit si cade
struct KnockableCone {
    float x, z;           // pozitie originala
    float curX, curZ;     // pozitie curenta
    float velX, velZ;     // viteza dupa lovire
    float fallAngle;      // unghi cadere
    float fallAxis[2];    // axa de cadere
    bool  knocked;        // a fost lovit?
    float knockTimer;     // timer respawn
};

// Coliziuni AABB (cladiri, tribune)
bool checkCarAABB(Car& car, const AABB& box);
void resolveCarBuilding(Car& car, const AABB& box);

// Coliziuni circulare (stalpi)
bool checkCarCircle(Car& car, float cx, float cz, float radius);
void resolveCarCircle(Car& car, float cx, float cz, float radius);

// Coliziuni masina-pieton
bool checkCarPedestrian(Car& car, Pedestrian& ped);

// Coliziuni masina-ambulanta
bool checkCarAmbulance(Car& car, Ambulance& amb);
void resolveCarAmbulance(Car& car, Ambulance& amb);

// Coliziuni masina-con (nu blocheaza, doar incetineste putin)
bool checkCarCone(Car& car, KnockableCone& cone);
void knockCone(Car& car, KnockableCone& cone);
void updateCones(std::vector<KnockableCone>& cones, float dt);

// Colectii de collidere
std::vector<AABB> getBuildingAABBs();
std::vector<KnockableCone> getKnockableCones();
std::vector<CircleCollider> getPoleColliders(const std::vector<LightPos>& poles);

#endif