#ifndef BC_TRAJECTORY_POINT_HPP
#define BC_TRAJECTORY_POINT_HPP

#include "BC_Unit.hpp"

class TrajectoryPoint
{
public:
    double time;
    Distance distance;
    Velocity velocity;  // 当前速度
    double mach;
    Distance height;  // 当前高度
    Distance drop;    // 掉落距离
    Angular dropAngle;  // 掉落角度,速度矢量方向
    Distance windage;   // 横风偏移距离
    Angular windageAngle;  // 横风偏移角度

    TrajectoryPoint(double t = 0, Distance d = Distance(), Velocity v = Velocity(), double m = 0,
                    Distance h = Distance(), Distance dr = Distance(), Angular da = Angular(), Distance w = Distance(),
                    Angular wa = Angular())
        : time(t), distance(d), velocity(v), mach(m), height(h), drop(dr), dropAngle(da), windage(w), windageAngle(wa)
    {
    }
};

#endif