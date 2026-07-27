#pragma once

struct Coord {
    float x;
    float y;

    Coord operator+(const Coord& o) const { return { x + o.x, y + o.y }; }
    Coord operator-(const Coord& o) const { return { x - o.x, y - o.y }; }
    Coord operator*(float s)        const { return { x * s,    y * s    }; }
    Coord operator/(float s)        const { return { x / s,    y / s    }; }
    bool  operator==(const Coord& o) const { return x == o.x && y == o.y; }
};

float length(Coord c);
Coord normalize(Coord c);