#ifndef BC_SIGHT_HPP
#define BC_SIGHT_HPP

#include <cmath>
#include <stdexcept>
#include <string>

#include "BC_Unit.hpp"
#include "BC_TrajectoryPoint.hpp"

// 焦平面类型枚举
enum class SightFocalPlane
{
    FFP,  // 第一焦平面
    SFP,  // 第二焦平面
    LWIR  // 长波红外
};

/**
 * @brief 瞄准镜刻度步长结构
 *
 * 表示水平和垂直方向的刻度调整步长
 */
struct SightReticleStep
{
    Angular horizontal;
    Angular vertical;

    SightReticleStep(Angular h = Angular::Mil(0.1), Angular v = Angular::Mil(0.1)) : horizontal(h), vertical(v)
    {
    }
};

/**
 * @brief 瞄准镜点击调整结构
 *
 * 表示水平和垂直方向需要调整的点击次数
 */
struct SightClicks
{
    double vertical;
    double horizontal;

    SightClicks(double v = 0, double h = 0) : vertical(v), horizontal(h)
    {
    }

    // 获取向上调整的点击数（正值表示向上）
    double getUpClicks() const
    {
        return vertical > 0 ? vertical : 0;
    }

    // 获取向下调整的点击数（正值表示向下）
    double getDownClicks() const
    {
        return vertical < 0 ? -vertical : 0;
    }

    // 获取向右调整的点击数（正值表示向右）
    double getRightClicks() const
    {
        return horizontal > 0 ? horizontal : 0;
    }

    // 获取向左调整的点击数（正值表示向左）
    double getLeftClicks() const
    {
        return horizontal < 0 ? -horizontal : 0;
    }

    std::string toString() const
    {
        char buffer[128];
        snprintf(buffer, sizeof(buffer), "SightClicks(Up:%.1f, Down:%.1f, Right:%.1f, Left:%.1f)", getUpClicks(),
                 getDownClicks(), getRightClicks(), getLeftClicks());
        return std::string(buffer);
    }
};

class Sight
{
private:
    SightFocalPlane focalPlane;
    Distance scaleFactor;
    Angular horizontalClickSize;
    Angular verticalClickSize;

public:
    /**
     * @brief 瞄准镜配置类，用于弹道计算和调整
     *
     * 这个类表示安装在武器上的光学瞄准镜系统，包括焦平面类型、放大倍率属性和点击调整大小。
     * 它提供基于目标距离和放大倍率设置计算瞄准镜调整的方法。
     */
    Sight(SightFocalPlane fp = SightFocalPlane::FFP, Distance sf = Distance::Meters(100),
          Angular hClick = Angular::Mil(0.2), Angular vClick = Angular::Mil(0.2))
        : focalPlane(fp), scaleFactor(sf), horizontalClickSize(hClick), verticalClickSize(vClick)
    {
        // 验证输入参数
        if (fp == SightFocalPlane::SFP && sf.Meters() <= 0)
        {
            throw std::invalid_argument("Scale factor required for SFP sights");
        }

        if (hClick.toMil() <= 0 || vClick.toMil() <= 0)
        {
            throw std::invalid_argument("Click sizes must be positive");
        }
    }

    // Getters
    SightFocalPlane getFocalPlane() const
    {
        return focalPlane;
    }
    Distance getScaleFactor() const
    {
        return scaleFactor;
    }
    Angular getHorizontalClickSize() const
    {
        return horizontalClickSize;
    }
    Angular getVerticalClickSize() const
    {
        return verticalClickSize;
    }

    // Setters
    void setFocalPlane(SightFocalPlane fp)
    {
        if (fp == SightFocalPlane::SFP && scaleFactor.Meters() <= 0)
        {
            throw std::invalid_argument("Scale factor must be set for SFP sights");
        }
        focalPlane = fp;
    }

    void setScaleFactor(Distance sf)
    {
        if (sf.Meters() <= 0)
        {
            throw std::invalid_argument("Scale factor must be positive");
        }
        scaleFactor = sf;
    }

    void setHorizontalClickSize(Angular clickSize)
    {
        if (clickSize.toMil() <= 0)
        {
            throw std::invalid_argument("Click size must be positive");
        }
        horizontalClickSize = clickSize;
    }

    void setVerticalClickSize(Angular clickSize)
    {
        if (clickSize.toMil() <= 0)
        {
            throw std::invalid_argument("Click size must be positive");
        }
        verticalClickSize = clickSize;
    }

    /**
     * @brief 计算SFP瞄准镜的刻度线步长
     *
     * 对于第二焦平面（SFP）瞄准镜，刻度线大小不随放大倍率变化，
     * 因此调整必须根据目标距离和放大倍率的关系进行相应缩放。
     *
     * @param targetDistance 目标距离
     * @param magnification 瞄准镜当前放大倍率
     * @return SightReticleStep 调整后的水平和垂直步长
     * @throws std::logic_error 如果不在SFP模式下调用
     * @throws std::invalid_argument 如果目标距离非正
     *
     * @example
     * SightReticleStep steps = sight.adjustSfpReticleSteps(
     *     Distance::Meters(300),  // 300米目标
     *     10.0                    // 10倍放大
     * );
     */
    SightReticleStep adjustSfpReticleSteps(Distance targetDistance, double magnification) const
    {
        if (focalPlane != SightFocalPlane::SFP)
        {
            throw std::logic_error("SFP focal plane required");
        }

        double targetDistM = targetDistance.Meters();
        if (targetDistM <= 0)
        {
            throw std::invalid_argument("Target distance must be positive");
        }

        // 计算SFP刻度线步长
        auto getSfpStep = [&](Angular clickSize) -> Angular {
            double scaleRatio = scaleFactor.Meters() / targetDistM;
            double adjustedValue = clickSize.toMil() * scaleRatio * magnification;
            return Angular::Mil(adjustedValue);
        };

        Angular hStep = getSfpStep(horizontalClickSize);
        Angular vStep = getSfpStep(verticalClickSize);

        return SightReticleStep(hStep, vStep);
    }

    /**
     * @brief 计算目标距离和放大倍率的瞄准镜调整
     *
     * 此方法基于给定目标距离和当前放大倍率的弹道解算计算所需的瞄准镜调整（点击次数）。
     * 计算方法根据焦平面类型而变化。
     *
     * @param targetDistance 目标距离
     * @param dropAngle 落点补偿所需的垂直角度调整
     * @param windageAngle 风偏补偿所需的水平角度调整
     * @param magnification 瞄准镜当前放大倍率
     * @return SightClicks 垂直和水平方向需要的点击调整次数
     * @throws std::invalid_argument 如果放大倍率非正或参数无效
     *
     * @note
     * - SFP瞄准镜：调整根据目标距离和放大倍率缩放
     * - FFP瞄准镜：直接使用点击大小转换
     * - LWIR瞄准镜：调整仅按放大倍率缩放
     *
     * @example
     * SightClicks clicks = sight.getAdjustment(
     *     Distance::Meters(500),      // 500米目标
     *     Angular::Mil(2.5),          // 2.5密位落点调整
     *     Angular::Mil(0.8),          // 0.8密位风偏调整
     *     12.0                         // 12倍放大
     * );
     * // clicks.vertical 向上调整的点击数
     * // clicks.horizontal 向右调整的点击数
     */
    SightClicks getAdjustment(Distance targetDistance, Angular dropAngle, Angular windageAngle,
                              double magnification) const
    {
        if (magnification <= 0)
        {
            throw std::invalid_argument("Magnification must be positive");
        }

        switch (focalPlane)
        {
            case SightFocalPlane::SFP: {
                SightReticleStep steps = adjustSfpReticleSteps(targetDistance, magnification);
                double vClicks = dropAngle.toMil() / steps.vertical.toMil();
                double hClicks = windageAngle.toMil() / steps.horizontal.toMil();
                return SightClicks(vClicks, hClicks);
            }

            case SightFocalPlane::FFP: {
                // FFP：直接使用点击大小转换
                double vClicks = dropAngle.toMil() / verticalClickSize.toMil();
                double hClicks = windageAngle.toMil() / horizontalClickSize.toMil();
                return SightClicks(vClicks, hClicks);
            }

            case SightFocalPlane::LWIR: {
                // LWIR：调整按放大倍率缩放
                double vClicks = dropAngle.toMil() / (verticalClickSize.toMil() / magnification);
                double hClicks = windageAngle.toMil() / (horizontalClickSize.toMil() / magnification);
                return SightClicks(vClicks, hClicks);
            }

            default:
                throw std::logic_error("Unknown focal plane type");
        }
    }

    /**
     * @brief 从弹道数据点计算瞄准镜调整
     *
     * 此便利方法从TrajectoryData实例提取必要的调整值并计算所需的瞄准镜点击次数。
     *
     * @param trajectoryPoint 包含弹道解算的TrajectoryData实例
     * @param magnification 瞄准镜当前放大倍率
     * @return SightClicks 垂直和水平方向需要的点击调整次数
     *
     * @example
     * // 假设trajectoryResult来自Calculator.fire()
     * for (const auto& point : trajectoryResult) {
     *     SightClicks clicks = sight.getTrajectoryAdjustment(point, 10.0);
     *     std::cout << "At " << point.getDistance().toMeters() << "m: "
     *               << clicks.getUpClicks() << " clicks up" << std::endl;
     * }
     */
    SightClicks getTrajectoryAdjustment(const TrajectoryPoint& trajectoryPoint, double magnification) const
    {
        return getAdjustment(trajectoryPoint.distance, trajectoryPoint.dropAngle, trajectoryPoint.windageAngle,
                             magnification);
    }

    /**
     * @brief 创建标准的FFP瞄准镜配置
     *
     * @param clickSize 点击大小（默认为0.1密位）
     * @return Sight FFP瞄准镜实例
     */
    static Sight createFFP(Angular clickSize = Angular::Mil(0.1))
    {
        return Sight(SightFocalPlane::FFP, Distance::Meters(100), clickSize, clickSize);
    }

    /**
     * @brief 创建标准的SFP瞄准镜配置
     *
     * @param scaleFactor 刻度因子距离
     * @param clickSize 点击大小（默认为0.25MOA）
     * @return Sight SFP瞄准镜实例
     */
    static Sight createSFP(Distance scaleFactor = Distance::Meters(100), Angular clickSize = Angular::MOA(0.25))
    {
        return Sight(SightFocalPlane::SFP, scaleFactor, clickSize, clickSize);
    }

    /**
     * @brief 创建标准的LWIR瞄准镜配置
     *
     * @param clickSize 点击大小（默认为0.5密位）
     * @return Sight LWIR瞄准镜实例
     */
    static Sight createLWIR(Angular clickSize = Angular::Mil(0.5))
    {
        return Sight(SightFocalPlane::LWIR, Distance::Meters(100), clickSize, clickSize);
    }

    /**
     * @brief 从字符串创建焦平面类型
     *
     * @param fpStr 焦平面字符串（"FFP", "SFP", "LWIR"）
     * @return SightFocalPlane 对应的枚举值
     * @throws std::invalid_argument 如果字符串无效
     */
    static SightFocalPlane focalPlaneFromString(const std::string& fpStr)
    {
        if (fpStr == "FFP" || fpStr == "ffp")
            return SightFocalPlane::FFP;
        if (fpStr == "SFP" || fpStr == "sfp")
            return SightFocalPlane::SFP;
        if (fpStr == "LWIR" || fpStr == "lwir")
            return SightFocalPlane::LWIR;
        throw std::invalid_argument("Invalid focal plane: " + fpStr);
    }

    /**
     * @brief 将焦平面类型转换为字符串
     *
     * @param fp 焦平面类型
     * @return 对应的字符串表示
     */
    static std::string focalPlaneToString(SightFocalPlane fp)
    {
        switch (fp)
        {
            case SightFocalPlane::FFP:
                return "FFP";
            case SightFocalPlane::SFP:
                return "SFP";
            case SightFocalPlane::LWIR:
                return "LWIR";
            default:
                return "UNKNOWN";
        }
    }

    /**
     * @brief 获取瞄准镜的字符串表示
     *
     * @return 描述瞄准镜配置的字符串
     */
    std::string toString() const
    {
        char buffer[256];
        snprintf(buffer, sizeof(buffer), "Sight(%s, Scale:%.1fm, HClick:%.3fmil, VClick:%.3fmil)",
                 focalPlaneToString(focalPlane).c_str(), scaleFactor.Meters(), horizontalClickSize.toMil(),
                 verticalClickSize.toMil());
        return std::string(buffer);
    }

    /**
     * @brief 检查瞄准镜配置是否有效
     *
     * @return true 如果配置有效
     * @return false 如果配置无效
     */
    bool isValid() const
    {
        if (horizontalClickSize.toMil() <= 0 || verticalClickSize.toMil() <= 0)
        {
            return false;
        }
        if (focalPlane == SightFocalPlane::SFP && scaleFactor.Meters() <= 0)
        {
            return false;
        }
        return true;
    }

    /**
     * @brief 比较两个瞄准镜配置是否相等
     *
     * @param other 要比较的另一个瞄准镜
     * @return true 如果配置相同
     * @return false 如果配置不同
     */
    bool operator==(const Sight& other) const
    {
        return focalPlane == other.focalPlane &&
               std::abs(scaleFactor.Meters() - other.scaleFactor.Meters()) < 1e-6 &&
               std::abs(horizontalClickSize.toMil() - other.horizontalClickSize.toMil()) < 1e-6 &&
               std::abs(verticalClickSize.toMil() - other.verticalClickSize.toMil()) < 1e-6;
    }

    bool operator!=(const Sight& other) const
    {
        return !(*this == other);
    }
};

#endif  // BC_SIGHT_HPP