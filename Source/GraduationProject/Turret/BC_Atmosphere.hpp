#ifndef BALLISTIC_CALC_IMPL_ATMOSPHERE_HPP
#define BALLISTIC_CALC_IMPL_ATMOSPHERE_HPP


#include "BC_Unit.hpp"


// 大气常数
constexpr double cStandardTemperatureF = 59.0;       // 标准温度(59°F = 15°C)
constexpr double cStandardTemperatureC = 15.0;       // 标准温度(15°C)
constexpr double cStandardPressureMetric = 1013.25;  // 标准气压(hPa)
constexpr double cStandardDensityMetric = 1.225;     // 标准空气密度(kg/m³)
constexpr double cStandardDensity = 0.0023769;       // 标准空气密度(lb/ft³)
constexpr double cStandardHumidity = 0.0;            // 标准湿度
constexpr double cLapseRateImperial = -0.00356616;   // 温度递减率(°F/ft)
constexpr double cLapseRateMetric = -0.0065;         // 温度递减率(K/m)
constexpr double cLapseRateKperFoot = -0.0019812;    // 温度递减率(K/ft)
constexpr double cDegreesCtoK = 273.15;              // 摄氏度转开尔文
constexpr double cDegreesFtoR = 459.67;              // 华氏度转兰金
constexpr double cPressureExponent = 5.255876;       // 气压公式指数
constexpr double cSpeedOfSoundImperial = 49.0223;    // 声速常数(英制)
constexpr double cSpeedOfSoundMetric = 20.0467;      // 声速常数(公制)
constexpr double cLowestTempF = -130.0;              // 最低模型温度(°F)
constexpr double cLowestTempC = -90.0;               // 最低模型温度(°C)


class Atmosphere
{
public:
    Distance altitude;  // 海拔高度
    Pressure pressure;  // 气压
    Temperature temperature;  // 温度
    double humidity;          // 相对湿度百分比
    Temperature powderTemp;   // 火药温度
    Velocity machMPS;         // 当地声速(米/秒)
    double densityRatio;      // 密度比（相对于标准密度）

private:

    double machFPS;             // 当地声速(英尺/秒)

    // 内部缓存的基础值
    double baseAltitudeFT;      // 基础海拔(英尺)
    double baseTempC;           // 基础温度(°C)
    double basePressureHPA;     // 基础气压(hPa)

    bool initializing;

public:
    Atmosphere(Distance alt = Distance(), Pressure press = Pressure::hPa(cStandardPressureMetric),
               Temperature temp = Temperature::Celsius(cStandardTemperatureC),
               double hum = 0, Temperature powder = Temperature::Celsius(cStandardTemperatureC))
        : altitude(alt), pressure(press), temperature(temp), humidity(hum), powderTemp(powder)
    {
        // 缓存基础值
        baseAltitudeFT = altitude.Feet();  // 海拔转换为英尺
        baseTempC = temperature.toCelsius();
        basePressureHPA = pressure.tohPa();

        // 计算当地声速
        machFPS = calculateMachF(temperature.toFahrenheit());
        machMPS = Velocity::FPS(machFPS);

        // 计算密度比
        updateDensityRatio();
        initializing = false;
    }

    /**
     * @brief 重新计算密度比（湿度变化时调用）
     */
    void updateDensityRatio()
    {
        double density = calculateAirDensity(baseTempC, basePressureHPA, humidity);
        densityRatio = density / cStandardDensityMetric;
    }

    /**
     * @brief 使用温度递减率在给定海拔插值温度
     *
     * @param altitudeFT 海拔高度（英尺）
     * @return 温度（摄氏度），受模型下限限制
     */
    double temperatureAtAltitude(double altitudeFT) const
    {
        double temp = (altitudeFT - baseAltitudeFT) * cLapseRateKperFoot + baseTempC;
        if (temp < cLowestTempC)
        {
            temp = cLowestTempC;
        }
        return temp;
    }

    /**
     * @brief 使用气压公式在给定海拔插值气压
     *
     * @param altitudeFT 海拔高度（英尺）
     * @return 气压（hPa）
     */
    double pressureAtAltitude(double altitudeFT) const
    {
        double tempK = baseTempC + cDegreesCtoK;
        double altitudeDiff = altitudeFT - baseAltitudeFT;

        return basePressureHPA * std::pow(1.0 + cLapseRateKperFoot * altitudeDiff / tempK, cPressureExponent);
    }

    /**
     * @brief 计算指定海拔的密度比和马赫数
     *
     * @param altitudeFT 海拔高度（英尺）
     * @return std::pair<double, double> (密度比, 马赫数-英尺/秒)
     */
    std::pair<double, double> getDensityAndMachForAltitude(double altitudeFT) const
    {
        // 快速路径：接近基础海拔时使用缓存值
        if (std::abs(baseAltitudeFT - altitudeFT) < 30.0)
        {
            return {densityRatio, machMPS.MPS()};
        }

        // 检查海拔是否超过对流层限制
        if (altitudeFT > 36089.0)
        {
            // 可以添加警告：海拔超过模型有效范围
        }

        // 计算温度和马赫数
        double tempC = temperatureAtAltitude(altitudeFT);
        double tempK = tempC + cDegreesCtoK;
        double mach = calculateMachK(tempK);

        // 计算气压和密度比
        double press = pressureAtAltitude(altitudeFT);
        double densityDelta = ((baseTempC + cDegreesCtoK) * press) / (basePressureHPA * tempK);
        double densityRatioAtAlt = densityRatio * densityDelta;

        return {densityRatioAtAlt, mach};
    }

    /**
     * @brief 获取指定海拔的密度比
     */
    double getDensityRatioAtAltitude(double altitudeFT) const
    {
        return getDensityAndMachForAltitude(altitudeFT).first;
    }

    /**
     * @brief 获取指定海拔的声速
     */
    Velocity getMachAtAltitude(double altitudeFT) const
    {
        auto result = getDensityAndMachForAltitude(altitudeFT);
        return Velocity::FPS(result.second);
    }


    // 静态方法：标准大气计算
    /**
     * @brief 获取指定海拔的标准温度（ICAO标准，有效至约36,000英尺）
     */
    static Temperature standardTemperature(Distance altitude)
    {
        double altitudeFT = altitude.Feet();
        double tempF = cStandardTemperatureF + altitudeFT * cLapseRateImperial;
        return Temperature::Fahrenheit(tempF);
    }

    /**
     * @brief 获取指定海拔的标准气压（ICAO标准，有效至约36,000英尺）
     */
    static Pressure standardPressure(Distance altitude)
    {
        double altitudeM = altitude.Meters();
        double tempK = cStandardTemperatureC + cDegreesCtoK;

        double pressure =
            cStandardPressureMetric * std::pow(1.0 + cLapseRateMetric * altitudeM / tempK, cPressureExponent);
        return Pressure::hPa(pressure);
    }

    /**
     * @brief 创建指定海拔的标准ICAO大气
     *
     * @param altitude 海拔高度
     * @param temperature 可选的覆盖温度（默认为标准温度）
     * @param humidity 相对湿度（小数或百分比）
     * @return Atmosphere 标准大气实例
     */
    static Atmosphere ICAO(Distance altitude = Distance(), Temperature temperature = Temperature(),
                           double humidity = cStandardHumidity)
    {
        Distance alt = altitude;
        Temperature temp = temperature;
        if (temp.toCelsius() == 0.0)
        {
            temp = standardTemperature(alt);
        }
        Pressure press = standardPressure(alt);
        return Atmosphere(alt, press, temp, humidity);
    }

    // 声速计算静态方法
    /**
     * @brief 根据华氏温度计算马赫1（英尺/秒）
     */
    static double calculateMachF(double fahrenheit)
    {
        if (fahrenheit < -cDegreesFtoR)
        {
            fahrenheit = cLowestTempF;  // 调整为最低有效温度
        }
        return std::sqrt(fahrenheit + cDegreesFtoR) * cSpeedOfSoundImperial;
    }

    /**
     * @brief 根据摄氏温度计算马赫1（米/秒）
     */
    static double calculateMachC(double celsius)
    {
        if (celsius < -cDegreesCtoK)
        {
            celsius = cLowestTempC;  // 调整为最低有效温度
        }
        return calculateMachK(celsius + cDegreesCtoK);
    }

    /**
     * @brief 根据开尔文温度计算马赫1（米/秒）
     */
    static double calculateMachK(double kelvin)
    {
        if (kelvin < 0)
        {
            kelvin = cLowestTempC + cDegreesCtoK;  // 调整为最低有效温度
        }
        return std::sqrt(kelvin) * cSpeedOfSoundMetric;
    }

    /**
     * @brief 根据温度、气压和湿度计算空气密度（CIPM-2007公式）
     *
     * @param tempC 温度（摄氏度）
     * @param pressureHPA 气压（hPa）
     * @param humidity 相对湿度（小数或百分比）
     * @return 空气密度（kg/m³）
     */
    static double calculateAirDensity(double tempC, double pressureHPA, double humidity)
    {
        const double R = 8.314472;       // 通用气体常数 J/(mol·K)
        const double M_a = 28.96546e-3;  // 干空气摩尔质量 kg/mol
        const double M_v = 18.01528e-3;  // 水蒸气摩尔质量 kg/mol

        // 饱和水汽压计算
        auto saturationVaporPressure = [](double T) -> double {
            const double A[] = {1.2378847e-5, -1.9121316e-2, 33.93711047, -6.3431645e3};
            return std::exp(A[0] * T * T + A[1] * T + A[2] + A[3] / T);
        };

        // 增强因子计算
        auto enhancementFactor = [](double p, double t) -> double {
            const double alpha = 1.00062;
            const double beta = 3.14e-8;
            const double gamma = 5.6e-7;
            return alpha + beta * p + gamma * t * t;
        };

        // 压缩因子计算
        auto compressibilityFactor = [](double p, double T, double x_v) -> double {
            const double a0 = 1.58123e-6;
            const double a1 = -2.9331e-8;
            const double a2 = 1.1043e-10;
            const double b0 = 5.707e-6;
            const double b1 = -2.051e-8;
            const double c0 = 1.9898e-4;
            const double c1 = -2.376e-6;
            const double d = 1.83e-11;
            const double e = -0.765e-8;

            double t_local = T - cDegreesCtoK;
            double Z = 1.0 -
                       (p / T) * (a0 + a1 * t_local + a2 * t_local * t_local + (b0 + b1 * t_local) * x_v +
                                  (c0 + c1 * t_local) * x_v * x_v) +
                       (p / T) * (p / T) * (d + e * x_v * x_v);

            return Z;
        };

        // 标准化湿度
        double rh = humidity;
        double rh_frac = (rh > 1.0) ? (rh / 100.0) : rh;
        rh_frac = std::max(0.0, std::min(1.0, rh_frac));

        // 单位转换
        double T_K = tempC + cDegreesCtoK;  // 开尔文
        double p = pressureHPA * 100.0;     // hPa -> Pa

        // 计算饱和水汽压和增强因子
        double p_sv = saturationVaporPressure(T_K);  // Pa
        double f = enhancementFactor(p, tempC);      // 增强因子

        // 水汽分压和摩尔分数
        double p_v = rh_frac * f * p_sv;  // Pa
        double x_v = p_v / p;             // 水汽摩尔分数

        // 计算压缩因子
        double Z = compressibilityFactor(p, T_K, x_v);

        // 最终密度计算
        return (p * M_a) / (Z * R * T_K) * (1.0 - x_v * (1.0 - M_v / M_a));
    }

    //cout 流
    friend std::ostream& operator<<(std::ostream& os, const Atmosphere& atmo)
    {
        os << "Altitude: " << atmo.altitude.Meters() << "m, Pressure: " << atmo.pressure.tohPa()
           << "hPa, Temperature: " << atmo.temperature.toCelsius() << "C, Humidity: " << atmo.humidity << "%";
        return os;
    }

};


#endif