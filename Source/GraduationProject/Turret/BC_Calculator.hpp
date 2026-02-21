#ifndef BC_CALCULATOR_HPP
#define BC_CALCULATOR_HPP


#include "BC_Unit.hpp"
#include "BC_DragModel.hpp"
#include "BC_Atmosphere.hpp"
#include "BC_Shot.hpp"
#include "BC_TrajectoryPoint.hpp"




enum class TerminationReason
{
    NONE,              // 正常终止
    MINIMUM_VELOCITY,  // 达到最小速度
    MAXIMUM_DROP,      // 达到最大下落
    MINIMUM_ALTITUDE,  // 达到最小海拔
    MAX_ITERATIONS     // 达到最大迭代次数
};

enum TrajFlag
{
    NONE = 0,
    RANGE = 1 << 0,  // 距离步长点
    TIME = 1 << 1,   // 时间步长点
    APEX = 1 << 2,   // 弹道顶点
    ALL = RANGE | TIME | APEX
};

/**
 * @brief 基础弹道数据点
 */
class BaseTrajData
{
public:
    double time;
    Vector3D position;
    Vector3D velocity;
    double mach;

    BaseTrajData(double t = 0, const Vector3D& pos = Vector3D(), const Vector3D& vel = Vector3D(), double m = 0)
        : time(t), position(pos), velocity(vel), mach(m)
    {
    }
};

class Calculator
{
public:
    // 内部配置常量
    static constexpr double cMinimumVelocity = 10.0;    // 最小速度阈值(m/s)
    static constexpr double cMaximumDrop = -1000.0;     // 最大下落距离(m)
    static constexpr double cMinimumAltitude = -100.0;  // 最小海拔(m)
    static constexpr double cGravity = 9.80665;         // 重力加速度(m/s²)
    static constexpr double cStandardDensity = 1.225;   // 标准空气密度(kg/m³)
    static constexpr double cSpeedOfSound = 340.0;      // 标准声速(m/s)

public:
    /**
     * @brief 计算给定射击参数的弹道
     *
     * @param shot 射击参数，包括位置和枪管角度
     * @param trajectory_range 停止计算弹道的距离
     * @param trajectory_step 记录弹道点之间的距离（可选，默认为trajectory_range）
     * @param time_step 记录点之间的最大时间间隔。如果>0，则至少按此频率记录点（默认0.0）
     * @param flags 特定关注点的标志（默认TrajFlag::NONE）
     * @param raise_range_error 如果为true，积分返回错误时抛出异常
     * @return std::unique_ptr<HitResult> 包含计算弹道的对象
     *
     * @throws std::invalid_argument 如果输入参数无效
     * @throws std::runtime_error 如果弹道计算失败且raise_range_error为true
     */
     //, 
    std::vector<TrajectoryPoint> fire(Shot& shot, Distance trajectory_range, Distance trajectory_step = Distance(),
        double time_step = 0.0, bool raise_range_error = true,
        TrajFlag flags = TrajFlag::NONE)
    {
        if (trajectory_range.Meters() <= 0)
        {
            throw std::invalid_argument("Trajectory range must be positive");
        }

        // 处理trajectory_step默认值
        Distance dist_step = trajectory_step;
        if (dist_step.Meters() <= 0)
        {
            dist_step = trajectory_range;
        }

        // 如果指定了步长，则添加范围标志
        TrajFlag filter_flags = flags;
        if (trajectory_step.Meters() > 0)
        {
            filter_flags = TrajFlag(filter_flags | TrajFlag::RANGE);
            if (dist_step.Meters() > trajectory_range.Meters())
            {
                dist_step = trajectory_range;
            }
        }

        // 转换单位
        double range_limit_m = trajectory_range.Meters();  // 最大计算距离以米为单位
        double range_step_m = dist_step.Meters();  // 距离步长以米为单位
        double time_step_sec = time_step;  // 时间步长以秒为单位

        // 执行积分计算
        auto result = integrate(shot, range_limit_m, range_step_m, time_step_sec, filter_flags);

        if (result.empty() && raise_range_error)
        {
            throw std::runtime_error("Trajectory integration error");
        }

        return result;
    }

    Vector3D calculateAcceleration(const Vector3D& relative_velocity, double km_coeff,
        const Shot& props, const Vector3D& ground_velocity)
    {

        Vector3D gravity(0, -cGravity, 0);

        // 阻力加速度 = -km * |v| * v
        Vector3D drag_acceleration = relative_velocity * (km_coeff * relative_velocity.magnitude());

        // 总加速度 = 重力加速度 - 阻力加速度
        Vector3D acceleration = gravity - drag_acceleration;

        if (props.latitudeValid && props.azimuthValid)
        {
            // 计算科里奥利加速度
            double latitude_rad = Angular::Degrees(props.latitude).toRadians();
            double azimuth_rad = Angular::Degrees(props.azimuth).toRadians();
            double omega = 7.2921159e-5;  // 地球自转角速度(弧度/秒)

            // 这个要在ENU 坐标系下计算
            const double vel_east = ground_velocity.z;
            const double vel_north = ground_velocity.x;
            const double vel_up = ground_velocity.y;

            const double factor = -2.0 * omega;
            const double sin_lat = sin(latitude_rad);
            const double cos_lat = cos(latitude_rad);

            const double accel_east = factor * (cos_lat * vel_up - sin_lat * vel_north);
            const double accel_north = factor * sin_lat * vel_east;
            const double accel_up = factor * (-cos_lat * vel_east);

            // 转回到NUE 坐标系
            double coriolis_x = accel_north;
            double coriolis_y = accel_up;
            double coriolis_z = accel_east;

            Vector3D coriolis_acceleration(coriolis_x, coriolis_y, coriolis_z);
            acceleration = acceleration + coriolis_acceleration;
        }
        return acceleration;
    }


    std::vector<TrajectoryPoint> integrate(const Shot& props, double range_limit_m, double range_step_m,
        double time_step_sec, TrajFlag filter_flags)  //
    {
        std::vector<TrajectoryPoint> trajectory;
        std::vector<BaseTrajData> step_data;  // 每一步3D位置和速度向量

        // 初始条件
        double time = 0.0;
        double dt = 0.001;  // 固定时间步长1ms


        // 瞄准镜的位置
        double cant_cosine = cos(props.cantAngle.toRadians());
        double cant_sin = sin(props.cantAngle.toRadians());
        double sight_height = props.weapon.sightHeight.Meters();

        // 初始位置
        Vector3D position(0, -cant_cosine * sight_height, -cant_sin * sight_height);  // 转换为米

        // 出膛速度向量
        double muzzle_velocity = props.ammo.muzzleVelocity.MPS();  // 或者子弹初速
        double elevation = props.getBarrelElevation().toRadians();   // 获取枪口朝向
        double azimuth = sin(props.cantAngle.toRadians()) * (props.weapon.zeroElevation.toRadians() + props.relativeAngle.toRadians());  //枪口相对于瞄准线的角度偏移

        Vector3D velocity = Vector3D(std::cos(elevation) * std::cos(azimuth),  // X 方向为正前方 North
            std::sin(elevation),                      // Y 方向为正上方 Up
            std::cos(elevation) * std::sin(azimuth))  // Z 方向为右侧   East
            * muzzle_velocity;

        // 先记录第一个点
        double initial_mach = velocity.magnitude() / cSpeedOfSound;
        trajectory.push_back(createTrajectoryData(time, position, velocity, props));


        // 假设只有一个风
        Vector3D windVector(0, 0, 0);
        if (props.winds.size() > 0)
        {
            auto& wind = props.winds[0];
            double windSpeed = wind.velocity.MPS();
            double windDirection = wind.direction.toRadians();  // 正北为0，顺时针为正方向

            // X 方向为正前方，  Y 正上方  Z 方向为右侧
            windVector = Vector3D(windSpeed * std::cos(windDirection), 0, windSpeed * std::sin(windDirection));
        }

        //大气参数
        double density_ratio = 1.0;     // 空气密度比
        double speed_of_sound = 340.0;  // 音速 m/s

        // 终止条件
        TerminationReason termination_reason = TerminationReason::NONE;

        int integration_step_count = 0;
        double last_record_time = 0.0;
        double last_record_distance = 0.0;

        // 弹道积分循环
        while (position.x <= range_limit_m && termination_reason == TerminationReason::NONE)
        {
            integration_step_count++;

            // 记录轨迹点（如果满足条件）
            bool should_record = false;
            if (filter_flags != TrajFlag::NONE)
            {
                // 距离步长记录
                if (position.x >= last_record_distance + range_step_m)
                {
                    should_record = true;
                }
                // 时间步长记录
                if (time_step_sec > 0 && time >= last_record_time + time_step_sec)
                {
                    should_record = true;
                }
                // 特殊点记录（顶点等）
                if (filter_flags & TrajFlag::APEX && velocity.y > 0)
                {
                    // 检查是否到达顶点
                    should_record = should_record || true;  // 简化实现
                }
            }
            else
            {
                // 如果没有指定过滤标志，记录所有点
                should_record = true;
            }

            if (should_record)
            {
                // 【修改】重命名为 record_mach 以避免遮挡
                double record_mach = velocity.magnitude() / cSpeedOfSound;
                trajectory.push_back(createTrajectoryData(time, position, velocity, props));
                last_record_time = time;
                last_record_distance = position.x;
            }

            // 计算当前海拔
            double current_altitude = props.atmo.altitude.Meters() + position.y;

            // 空气密度比例  声速(m/s)
            // 【修改】重命名为 step_density_ratio 和 step_speed_of_sound 以避免遮挡
            auto [step_density_ratio, step_speed_of_sound] =
                props.atmo.getDensityAndMachForAltitude(current_altitude / 0.3048);  // 转换为英尺

            // 计算相对风的速度
            Vector3D relative_velocity = velocity - windVector;
            double relative_speed = relative_velocity.magnitude();

            // 计算马赫数和阻力系数
            //density_ratio 当前空气和标准空气密度的比例
            // 【修改】重命名为 calc_mach，并使用新变量名 step_speed_of_sound
            double calc_mach = relative_speed / step_speed_of_sound;
            double cd = props.ammo.dragModel.getDragCoefficient(calc_mach);  //G1 表
            // 【修改】使用新变量名 step_density_ratio
            double km = step_density_ratio * cd * 2.08551e-04 / props.ammo.dragModel.bc;

            km *= 3.28084;   //测试量纲

            // ... (注释略) ...

            Vector3D k1_velocity, k2_velocity, k3_velocity, k4_velocity;
            Vector3D k1_position, k2_position, k3_position, k4_position;

            // k1阶段
            k1_velocity = calculateAcceleration(relative_velocity, km, props, velocity);
            k1_position = velocity;

            // k2阶段
            Vector3D temp_velocity = relative_velocity + k1_velocity * (0.5 * dt);
            k2_velocity = calculateAcceleration(temp_velocity, km, props, velocity + k1_velocity * (0.5 * dt));
            k2_position = velocity + k1_velocity * (0.5 * dt);

            // k3阶段
            temp_velocity = relative_velocity + k2_velocity * (0.5 * dt);
            k3_velocity = calculateAcceleration(temp_velocity, km, props, velocity + k2_velocity * (0.5 * dt));
            k3_position = velocity + k2_velocity * (0.5 * dt);

            // k4阶段
            temp_velocity = relative_velocity + k3_velocity * dt;
            k4_velocity = calculateAcceleration(temp_velocity, km, props, velocity + k3_velocity * dt);
            k4_position = velocity + k3_velocity * dt;

            // 更新速度和位置
            Vector3D velocity_increment =
                (k1_velocity + k2_velocity * 2.0 + k3_velocity * 2.0 + k4_velocity) * (dt / 6.0);
            Vector3D position_increment =
                (k1_position + k2_position * 2.0 + k3_position * 2.0 + k4_position) * (dt / 6.0);

            velocity = velocity + velocity_increment;
            position = position + position_increment;
            time += dt;

            // 检查终止条件
            termination_reason = checkTermination(position, velocity, props);

            // 防止无限循环
            if (integration_step_count > 100000)
            {
                termination_reason = TerminationReason::MAX_ITERATIONS;
                break;
            }
        }

        // 记录最终点
        if (termination_reason != TerminationReason::NONE)
        {
            trajectory.push_back(createTrajectoryData(time, position, velocity, props));
        }
        return trajectory;
    }


    /**
     * @brief 创建轨迹数据点
     */
    TrajectoryPoint createTrajectoryData(double time, const Vector3D& position, const Vector3D& velocity,
        const Shot& props)
    {
        double mach = velocity.magnitude() / cSpeedOfSound;
        Distance distance = Distance::Meters(position.x);
        Distance spin_drift = Distance::Inches(props.spin_drift(time));   //由于膛线导致的偏移，会和风偏叠加在一起
        double drop = distance.Meters() * tan(props.getBarrelElevation().toRadians()) - position.y;  //相对于枪口指向的下落
        double windage = position.z + spin_drift.Meters();
        Angular windage_angle = Angular::Radians(std::atan2(windage, position.x));
        return TrajectoryPoint(time, distance,                      // 射击距离
            Velocity::MPS(velocity.magnitude()), // 速度
            mach,                                // 马赫数
            Distance::Meters(position.y),        // height
            drop,                                // drop 同x情况下，相对于枪管射线的下落
            Angular::Radians(std::atan2(velocity.y, velocity.x)),  // drop angle 速度矢量方向
            windage,                                               // 风偏 有问题
            windage_angle                                          // windage angle
        );
    }

    /**
     * @brief 检查终止条件
     */
    TerminationReason checkTermination(const Vector3D& position, const Vector3D& velocity, const Shot& props)
    {
        double speed = velocity.magnitude();
        double altitude = props.atmo.altitude.Meters() + position.y;  // 转换为米
        //props.getAlt0Ft() * 0.3048 + position.y;  // 转换为米

        if (speed < cMinimumVelocity)
        {
            return TerminationReason::MINIMUM_VELOCITY;
        }
        if (position.y < cMaximumDrop)   // 最大下落
        {
            return TerminationReason::MAXIMUM_DROP;
        }
        if (altitude < cMinimumAltitude)  // 最小海拔
        {
            return TerminationReason::MINIMUM_ALTITUDE;
        }

        return TerminationReason::NONE;
    }
};



#endif