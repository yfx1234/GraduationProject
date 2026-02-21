#ifndef BALLISTIC_CALC_IMPL_AMMUNITION_HPP
#define BALLISTIC_CALC_IMPL_AMMUNITION_HPP

#include "BC_Unit.hpp"
#include "BC_DragModel.hpp"

class Ammunition
{
public:
    DragModel dragModel;        // 弹道阻力模型
    Velocity muzzleVelocity;    // 枪口初速
    Temperature powderTemp;     // 火药温度
    double tempModifier;        // 温度修正系数（每15摄氏度变化的百分比）
    bool usePowderSensitivity;  // 是否使用火药温度敏感性

public:
    Ammunition(const DragModel& dm, Velocity mv, Temperature pt = Temperature(15), double tm = 0, bool ups = false)
        : dragModel(dm), muzzleVelocity(mv), powderTemp(pt), tempModifier(tm), usePowderSensitivity(ups)
    {
        if (muzzleVelocity.MPS() <= 0)
        {
            throw std::invalid_argument("Muzzle velocity must be positive");
        }
    }

    /**
     * @brief 计算火药温度敏感性系数并更新tempModifier
     *
     * 此方法基于两个已知的速度/温度数据点计算火药温度敏感性系数，
     * 并将结果赋给tempModifier属性。
     *
     * @param otherVelocity 在otherTemperature温度下的已知速度
     * @param otherTemperature 对应otherVelocity的温度
     * @return 温度修正系数（每15°C速度变化的百分比）
     * @throws std::invalid_argument 如果速度或温度差为零
     *
     * @note 计算公式：
     * tempModifier = (velocity_delta / temperature_delta) * (15 / lower_velocity)
     * 这提供了一个标准化的敏感性值，表示每15°C温度变化的速度百分比变化。
     *
     * @example
     * // 从已知的冷天速度下降计算敏感性
     * double sensitivity = ammo.calcPowderSens(
     * Velocity::MPS(800),  // 冷天速度
     * Temperature::Celsius(0)  // 冷天温度
     * );
     */
    double calcPowderSens(Velocity otherVelocity, Temperature otherTemperature)
    {
        double v0 = muzzleVelocity.MPS();
        double t0 = powderTemp.toCelsius();
        double v1 = otherVelocity.MPS();
        double t1 = otherTemperature.toCelsius();

        if (v0 <= 0 || v1 <= 0)
        {
            throw std::invalid_argument("calcPowderSens requires positive muzzle velocities");
        }

        double v_delta = std::abs(v0 - v1);
        double t_delta = std::abs(t0 - t1);
        double v_lower = (v1 < v0) ? v1 : v0;

        if (v_delta < 1e-10 || t_delta < 1e-10)
        {
            throw std::invalid_argument("Velocity and temperature differences cannot be zero");
        }

        tempModifier = (v_delta / t_delta) * (15.0 / v_lower);
        return tempModifier;
    }

    /**
     * @brief 计算基于火药温度调整后的初速
     *
     * 此方法基于基准速度、火药温度和温度敏感性系数计算给定温度下的初速。
     * 如果火药敏感性被禁用，则返回基准速度。
     *
     * @param currentTemp 弹药火药的当前温度
     * @return 根据温度修正后的初速
     *
     * @note 计算公式：
     * adjusted_velocity = baseline_velocity + (temp_modifier * baseline_velocity * temp_delta) / 15.0
     * 其中temp_delta是currentTemp和powderTemp之间的差值。
     *
     * 如果usePowderSensitivity为false，则忽略温度返回基准初速。
     *
     * @example
     * // 获取当前条件下的速度
     * Velocity coldVelocity = ammo.getVelocityForTemp(Temperature::Celsius(-10));
     * Velocity hotVelocity = ammo.getVelocityForTemp(Temperature::Celsius(35));
     *
     * // 禁用火药敏感性
     * ammo.setUsePowderSensitivity(false);
     * Velocity constantVelocity = ammo.getVelocityForTemp(Temperature::Celsius(-10));
     * // constantVelocity 等于基准速度，忽略温度
     */
    Velocity getVelocityForTemp(Temperature currentTemp) const
    {
        if (!usePowderSensitivity)
        {
            return muzzleVelocity;
        }

        try
        {
            double v0 = muzzleVelocity.MPS();
            double t0 = powderTemp.toCelsius();
            double t1 = currentTemp.toCelsius();
            double t_delta = t1 - t0;

            // 修正公式：adjusted_velocity = v0 + (temp_modifier * v0 * t_delta) / 15.0
            double muzzle_velocity = v0 + (tempModifier * v0 * t_delta) / 15.0;

            // 确保速度不为负
            if (muzzle_velocity <= 0)
            {
                return muzzleVelocity;  // 返回基准速度作为安全值
            }

            return Velocity::MPS(muzzle_velocity);
        }
        catch (const std::exception&) // 【修改】移除了未使用的变量 e
        {
            // 发生错误时返回基准速度
            return muzzleVelocity;
        }
    }
};

#endif