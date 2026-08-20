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


    

