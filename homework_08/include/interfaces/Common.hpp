#pragma once
#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>


struct Coord
{
    float x;
    float y;

    Coord operator+(const Coord& o) const { return { x + o.x, y + o.y }; }
    Coord operator-(const Coord& o) const { return { x - o.x, y - o.y }; }
    Coord operator*(float s)        const { return { x * s,    y * s    }; }
    Coord operator/(float s)        const { return { x / s,    y / s    }; }
    bool  operator==(const Coord& o) const { return x == o.x && y == o.y; }
};

inline float length(Coord c)    { return std::hypot(c.x, c.y); } //Computes the square root of the sum of the squares of x and y
inline Coord normalize(Coord c) { float l = length(c); return l > 1e-6f ? c / l : Coord{0, 0}; }

struct AmmoParams
{
    char  name[32];
    float mass, drag, lift;
    
};

struct DroneConfig
{
    Coord startPos;
    float altitude;
    float initialDir;
    float attackSpeed;
    float accelPath;
    char  ammoName[32];
    float arrayTimeStep;
    float simTimeStep;
    float hitRadius;
    float angularSpeed;
    float turnThreshold;
};

struct SimStep
{
    Coord pos;
    float direction;
    int   state;
    int   targetIdx;
    Coord dropPoint;    // куди летить дрон (точка скиду / fire point)
    Coord aimPoint;     // куди впаде бомба якщо скинути зараз
    Coord predictedTarget; // прогнозована позиція цілі
};

struct TargetTrajectory {
    std::vector<Coord>positions;
};

enum DroneState {
    STOPPED = 0,
    ACCELERATING = 1,
    MOVING = 2,
    TURNING = 3,
    DROPPED = 4
};
