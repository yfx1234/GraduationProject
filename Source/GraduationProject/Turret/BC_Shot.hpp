#ifndef BC_SHOT_HPP
#define BC_SHOT_HPP

#include "BC_Unit.hpp"
#include "BC_Wind.hpp"
#include "BC_Ammunition.hpp"
#include "BC_Atmosphere.hpp"
#include "BC_Weapon.hpp"

class Shot
{
public:
    Ammunition ammo;
    Atmosphere atmo;
    Weapon weapon;
    std::vector<Wind> winds;
    Angular lookAngle;
    Angular relativeAngle;
    Angular cantAngle;
    double azimuth;
    double latitude;

    bool azimuthValid = false;
    bool latitudeValid = false;

    double stability_coefficient = 0.0;  //Litz spin-drift approximation.

    /**
     * @brief 构造函数
     *
     * @param a 弹药配置
     * @param atm 大气条件
     * @param w 武器配置
     * @param wds 风层列表
     * @param la look_angle 瞄准线角度
     * @param ra relative_angle 相对角度（抬高调整）
     * @param ca cant_angle倾斜角度
     * @param az 方位角[0, 360)，可选  正北为0，顺时针为正方向，计算科里奥利效应时使用
     * @param lat 纬度[-90, 90]，可选  计算科里奥利效应时使用
     */
    Shot(const Ammunition& a, const Atmosphere& atm, const Weapon& w, const std::vector<Wind>& wds = {},
         Angular la = Angular(), Angular ra = Angular(), Angular ca = Angular(), double az = 0, double lat = 0)
        : ammo(a),
          atmo(atm),
          weapon(w),
          winds(wds),
          lookAngle(la),
          relativeAngle(ra),
          cantAngle(ca),
          azimuth(az),
          latitude(lat),
          azimuthValid(false),
          latitudeValid(false)
    {
        if (az < 0.0 || az >= 360.0)
        {
            azimuthValid = false;
            throw std::invalid_argument("Azimuth must be in range [0, 360)");
        }
        if (lat < -90.0 || lat > 90.0)
        {
            latitudeValid = false;
            throw std::invalid_argument("Latitude must be in range [-90, 90]");
        }
        // 考虑子弹螺旋导致的横偏
        stability_coefficient = _calc_stability_coefficient();
    }


    /**
     * @brief 获取枪管方位角（相对于瞄准线的水平角度）
     *
     * @return 枪管方位角
     */
    Angular getBarrelAzimuth() const
    {
        double cantRad = cantAngle.toRadians();
        double zeroElevRad = weapon.zeroElevation.toRadians();
        double relativeRad = relativeAngle.toRadians();
        return Angular::Radians(std::sin(cantRad) * (zeroElevRad + relativeRad));
    }

    /**
     * @brief 获取枪管仰角（垂直平面中相对于水平面的总仰角）
     *
     * 计算公式：look_angle + cos(cant_angle) * (zero_elevation + relative_angle)
     *
     * @return 枪管仰角
     */
    Angular getBarrelElevation() const
    {
        double lookRad = lookAngle.toRadians();
        double cantRad = cantAngle.toRadians();
        double zeroElevRad = weapon.zeroElevation.toRadians();
        double relativeRad = relativeAngle.toRadians();
        return Angular::Radians(lookRad + std::cos(cantRad) * (zeroElevRad + relativeRad));
    }

    /**
     * @brief 设置枪管仰角（通过调整relative_angle实现）
     *
     * @param elevation 期望的枪管仰角
     */
    void setBarrelElevation(Angular elevation)
    {
        double elevRad = elevation.toRadians();
        double lookRad = lookAngle.toRadians();
        double cantRad = cantAngle.toRadians();
        double zeroElevRad = weapon.zeroElevation.toRadians();

        double relativeRad = elevRad - lookRad - std::cos(cantRad) * zeroElevRad;
        relativeAngle = Angular::Radians(relativeRad);
    }

public:
    /**
     * Calculate the Miller stability coefficient.
     *
     * @return The Miller stability coefficient.
     */
    double _calc_stability_coefficient() const
    {
        // 获取基本尺寸参数（转换为英寸）
        double twist_inch = weapon.twist.Inches();
        double length_inch = ammo.dragModel.length.Inches();   //子弹长度
        double diameter_inch = ammo.dragModel.diameter.Inches();   //子弹直径
        double pressure = atmo.pressure.tohPa();
        double weight = ammo.dragModel.weight.toGrains();
        double mv = ammo.muzzleVelocity.FPS();

        // 检查必要参数是否有效
        if (abs(twist_inch) > 0.0001 && abs(length_inch) > 0.0001 && abs(diameter_inch) > 0.0001)
        {
            // 计算缠距比和长径比
            double twist_rate = std::fabs(twist_inch) / diameter_inch;
            double length_ratio = length_inch / diameter_inch;

            // Miller稳定性公式主计算
            double sd = (30.0 * weight) / (std::pow(twist_rate, 2.0) * std::pow(diameter_inch, 3.0) *
                                                         length_ratio * (1.0 + std::pow(length_ratio, 2.0)));

            // 初速修正因子
            double fv = std::pow(mv / 2800.0, 1.0 / 3.0);

            // 大气条件修正
            double ft = atmo.temperature.toFahrenheit();  // 转换为华氏度
            double pt = atmo.pressure.toInHg();           // 转换为英寸汞柱

            // 温度压力修正因子
            double ftp = ((ft + 460.0) / (59.0 + 460.0)) * (29.92 / pt);

            return sd * fv * ftp;
        }
        return 0.0;
    }

    /**
     * Litz spin-drift approximation.
     * 计算子弹因自旋导致的横向偏移
     * @param time Time of flight
     * @return Windage due to spin drift, in inches
     */
    double spin_drift(float time) const
    {
        double twist_inch = weapon.twist.Inches();
        if ((stability_coefficient != 0) && (twist_inch != 0))
        {
            double sign = (twist_inch > 0) ? 1.0 : -1.0;
            return sign * (1.25 * (stability_coefficient + 1.2) * std::pow(time, 1.83)) / 12.0;
        }
        return 0.0;
    }
};

#endif