#include "collision.h"
#include "ambulance.h"
#include <math.h>

// Verifica daca un punct e in AABB
static bool pointInAABB(float px, float pz, const AABB& box) {
    return px >= box.minX && px <= box.maxX && pz >= box.minZ && pz <= box.maxZ;
}

// Verifica coliziune masina (4 colturi) cu AABB
bool checkCarAABB(Car& car, const AABB& box) {
    for (int i = 0; i < 4; i++) {
        if (pointInAABB(car.corners[i][0], car.corners[i][1], box))
            return true;
    }
    if (pointInAABB(car.x, car.z, box))
        return true;
    return false;
}

// Verifica coliziune masina cu pieton (distanta centru-centru)
bool checkCarPedestrian(Car& car, Pedestrian& ped) {
    float dx = car.x - ped.x;
    float dz = car.z - ped.z;
    float dist = sqrtf(dx * dx + dz * dz);
    float collDist = 1.5f;
    return dist < collDist;
}

// Coliziune masina cu obiect circular (conuri, stalpi)
bool checkCarCircle(Car& car, float cx, float cz, float radius) {
    // Verificam distanta centru masina - centru cerc
    float dx = car.x - cx;
    float dz = car.z - cz;
    float dist = sqrtf(dx * dx + dz * dz);
    float carRadius = fmaxf(car.width, car.length) * 0.45f;
    return dist < (radius + carRadius);
}

// Rezolva coliziune cu obiect circular - impinge masina afara
void resolveCarCircle(Car& car, float cx, float cz, float radius) {
    float dx = car.x - cx;
    float dz = car.z - cz;
    float dist = sqrtf(dx * dx + dz * dz);
    if (dist < 0.01f) { dx = 1; dist = 1; }  // evita div by 0
    float carRadius = fmaxf(car.width, car.length) * 0.45f;
    float overlap = (radius + carRadius) - dist;
    if (overlap > 0) {
        car.x += (dx / dist) * (overlap + 0.1f);
        car.z += (dz / dist) * (overlap + 0.1f);
    }
    car.speed *= 0.5f;
    car.updateCorners();
}

// Coliziune masina cu ambulanta
bool checkCarAmbulance(Car& car, Ambulance& amb) {
    float dx = car.x - amb.x;
    float dz = car.z - amb.z;
    float dist = sqrtf(dx * dx + dz * dz);
    float collDist = (car.length + amb.length) * 0.4f;
    return dist < collDist;
}

void resolveCarAmbulance(Car& car, Ambulance& amb) {
    float dx = car.x - amb.x;
    float dz = car.z - amb.z;
    float dist = sqrtf(dx * dx + dz * dz);
    if (dist < 0.01f) { dx = 1; dist = 1; }
    float push = ((car.length + amb.length) * 0.4f) - dist + 0.3f;
    if (push > 0) {
        car.x += (dx / dist) * push;
        car.z += (dz / dist) * push;
    }
    car.speed *= 0.15f;
    car.updateCorners();
}

// Rezolva coliziune cu AABB (cladiri, tribune, billboard-uri)
void resolveCarBuilding(Car& car, const AABB& box) {
    float cx = car.x, cz = car.z;
    float dLeft = fabsf(cx - box.minX);
    float dRight = fabsf(cx - box.maxX);
    float dFront = fabsf(cz - box.minZ);
    float dBack = fabsf(cz - box.maxZ);

    float minD = dLeft;
    int side = 0;
    if (dRight < minD) { minD = dRight; side = 1; }
    if (dFront < minD) { minD = dFront; side = 2; }
    if (dBack < minD) { minD = dBack;  side = 3; }

    float push = 0.3f;
    switch (side) {
    case 0: car.x = box.minX - car.width * 0.5f - push; break;
    case 1: car.x = box.maxX + car.width * 0.5f + push; break;
    case 2: car.z = box.minZ - car.length * 0.5f - push; break;
    case 3: car.z = box.maxZ + car.length * 0.5f + push; break;
    }
    car.speed *= 0.1f;
    car.updateCorners();
}

// Toate AABB-urile din scena: cladiri + tribune (FARA billboard-uri - sunt prea subtiri)
std::vector<AABB> getBuildingAABBs() {
    std::vector<AABB> boxes;

    // === CLADIRI ===
    boxes.push_back({ -75, -15, -75 + 12, -15 + 8 });
    boxes.push_back({ -75,  10, -75 + 10,  10 + 7 });
    boxes.push_back({ 62, -15,  62 + 10, -15 + 8 });
    boxes.push_back({ 62,  10,  62 + 8,   10 + 7 });
    boxes.push_back({ -20, -75, -20 + 14, -75 + 6 });
    boxes.push_back({ 10, -75,  10 + 12, -75 + 6 });

    // === TRIBUNE (AABB-uri mai stranse, sa nu blocheze drumul) ===
    // Tribuna dreapta: drawTribune(55, -8, 18, 6, 90) - rotita 90, creste spre +X
    { boxes.push_back({ 56, -8, 56 + 7.0f, -8 + 18 }); }
    // Tribuna stanga: drawTribune(-60, -5, 16, 5, -90) - rotita -90, creste spre -X
    { boxes.push_back({ -60 - 6.0f, -5, -60, -5 + 16 }); }
    // Tribuna sus: drawTribune(-15, 58, 20, 4, 180) - rotita 180, creste spre -Z
    { boxes.push_back({ -15, 58, -15 + 20, 58 + 5.0f }); }
    // Tribuna jos: drawTribune(-10, -62, 18, 4, 0) - rotita 0, creste spre +Z
    { boxes.push_back({ -10, -62, -10 + 18, -62 + 5.0f }); }

    return boxes;
}

// Conuri care pot fi lovite si cad
std::vector<KnockableCone> getKnockableCones() {
    std::vector<KnockableCone> cones;
    float coneData[][2] = {
        {-12,46},{-14,44},{-16,42},{-18,40},{-16,35},{-14,33},
        {-52,8},{-54,0},{-52,-8},{-48,-16},
        {-25,-48},{-10,-53},{8,-53},{25,-50},
        {47,35},{42,46},{32,50},{18,52}
    };
    for (int i = 0; i < 18; i++) {
        KnockableCone c;
        c.x = coneData[i][0]; c.z = coneData[i][1];
        c.curX = c.x; c.curZ = c.z;
        c.velX = 0; c.velZ = 0;
        c.fallAngle = 0;
        c.fallAxis[0] = 1; c.fallAxis[1] = 0;
        c.knocked = false;
        c.knockTimer = 0;
        cones.push_back(c);
    }
    return cones;
}

// Verifica coliziune masina-con
bool checkCarCone(Car& car, KnockableCone& cone) {
    if (cone.knocked) return false;  // deja cazut
    float dx = car.x - cone.curX;
    float dz = car.z - cone.curZ;
    float dist = sqrtf(dx * dx + dz * dz);
    return dist < 1.8f;  // raza mica
}

// Loveste conul - il impinge in directia masinii
void knockCone(Car& car, KnockableCone& cone) {
    float dx = cone.curX - car.x;
    float dz = cone.curZ - car.z;
    float dist = sqrtf(dx * dx + dz * dz);
    if (dist < 0.01f) { dx = 1; dist = 1; }

    // Conul zboara in directia impactului
    float pushSpeed = fabsf(car.speed) * 0.4f + 2.0f;
    cone.velX = (dx / dist) * pushSpeed;
    cone.velZ = (dz / dist) * pushSpeed;
    cone.knocked = true;
    cone.knockTimer = 8.0f;  // respawn dupa 8 sec
    cone.fallAxis[0] = dx / dist;
    cone.fallAxis[1] = dz / dist;
    cone.fallAngle = 0;

    // Masina incetineste PUTIN (nu se opreste)
    car.speed *= 0.92f;
}

// Update conuri - miscare dupa lovire si respawn
void updateCones(std::vector<KnockableCone>& cones, float dt) {
    for (auto& c : cones) {
        if (!c.knocked) continue;

        // Miscare cu decelerare
        c.curX += c.velX * dt;
        c.curZ += c.velZ * dt;
        c.velX *= 0.95f;
        c.velZ *= 0.95f;

        // Cadere animata
        if (c.fallAngle < 90.0f) {
            c.fallAngle += 300.0f * dt;
            if (c.fallAngle > 90.0f) c.fallAngle = 90.0f;
        }

        // Timer respawn
        c.knockTimer -= dt;
        if (c.knockTimer <= 0) {
            c.knocked = false;
            c.curX = c.x;
            c.curZ = c.z;
            c.velX = 0; c.velZ = 0;
            c.fallAngle = 0;
        }
    }
}

// Colectia de stalpi de iluminat
std::vector<CircleCollider> getPoleColliders(const std::vector<LightPos>& poles) {
    std::vector<CircleCollider> colliders;
    for (auto& p : poles) {
        colliders.push_back({ p.x, p.z, 0.4f });
    }
    return colliders;
}