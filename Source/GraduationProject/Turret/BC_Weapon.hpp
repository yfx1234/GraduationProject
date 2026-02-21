#ifndef BC_WEAPON_HPP
#define BC_WEAPON_HPP

#include "BC_Unit.hpp"

#include "BC_Sight.hpp"

// 武器参数
class Weapon
{
public:
    Distance sightHeight;  // 瞄准镜高度
    Distance twist;        // 枪管膛线旋转一周距离
    Angular zeroElevation;  // 瞄准镜水平时，枪管的仰角
    Sight sight;            // 瞄准镜参数

public:
    Weapon(Distance sh = Distance::Inches(0), Distance tw = Distance::Inches(0), Angular ze = Angular::Degrees(0),
    Sight sight = Sight())
        : sightHeight(sh), twist(tw), zeroElevation(ze)
    {
    }
};

#endif  // BC_WEAPON_HPP