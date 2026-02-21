#ifndef BC_WIND_HPP
#define BC_WIND_HPP

#include "BC_Unit.hpp"

class Wind
{
public:
    Velocity velocity;  // 风速
    // direction_from: 0 is blowing from behind shooter.
    // 90 degrees is blowing from shooter's left towards right.
    Angular direction;  // 风向，角度，0度为正北方向，顺时针为正方向
    Distance untilDistance;  // 风作用的距离范围,如果不指定，则视为无限远


public:
    Wind(Velocity vel = Velocity(), Angular dir = Angular(), Distance dist = Distance(1e8))
        : velocity(vel), direction(dir), untilDistance(dist)
    {

    }

    // 获取风的范围 (range, 0, cross)
    Vector3D getVector() const
    {
        double speed = velocity.MPS();
        double dir = direction.toRadians();
        return Vector3D(speed * std::cos(dir),  // x分量
                        0,                      // y分量
                        speed * std::sin(dir)   // z分量
        );
    }
};

#endif