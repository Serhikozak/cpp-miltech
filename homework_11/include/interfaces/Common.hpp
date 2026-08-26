#pragma once

struct Coord
{
    float x;
    float y;

};

struct Target {
    Coord pos;
    Coord velocity;
};

struct DroneCommand {
    int state; //новий стан для дрона
    float angelSpeed; //кутова швидкість повороту
    
};

struct DroneTelemetry {
    Coord pos;
    Coord speed;
    
    float timeSecSinceStart;
};

struct AmmoParams
{
    char  name[32];
    float mass, drag, lift;
};

struct DroneConfig
{
    float hitRadius;
    float turnThreshold;
};
enum DroneState {
    STOPPED = 0,
    ACCELERATING = 1,
    MOVING = 2,
    TURNING = 3,
    DROPPED = 4
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


    

