//#ifndef ITargetProvider_H
//#define ITargetProvider_H
#pragma once
#include "Common.hpp"
class ITargetProvider {
public:
    virtual int getTargetCount()        = 0;
    virtual Coord getTarget(int index) = 0;
    virtual ~ITargetProvider() = default;
};