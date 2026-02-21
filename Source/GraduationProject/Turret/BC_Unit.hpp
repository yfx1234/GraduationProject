#ifndef BALLISTIC_CALC_IMPL_UNIT_HPP
#define BALLISTIC_CALC_IMPL_UNIT_HPP

#include <cmath>
#include <ostream>

// 距离类 - 内部使用米(m)作为基本单位
class Distance
{
private:
    double value_meters;

public:
    Distance(double meters = 0) : value_meters(meters)
    {
    }

    //mm
    static Distance Millimeters(double mm)  // 1毫米 = 0.001米
    {
        return Distance(mm * 0.001);
    }

    static Distance Centimeters(double cm)  // 1厘米 = 0.01米
    {
        return Distance(cm * 0.01);
    }

    static Distance Meters(double meters)
    {
        return Distance(meters);
    }
    static Distance Feet(double feet)  // 1 foot = 0.3048 meters 英尺
    {
        return Distance(feet * 0.3048);
    }
    static Distance Inches(double inches)  // 1 inch = 0.0254 meters 英寸
    {
        return Distance(inches * 0.0254);
    }

    double toMillimeters() const
    {
        return value_meters * 1000.0;
    }

    double toCentimeters() const
    {
        return value_meters * 100.0;
    }

    double Meters() const
    {
        return value_meters;
    }
    double Feet() const  // 英尺
    {
        return value_meters / 0.3048;
    }
    double Inches() const   // 英寸
    {
        return value_meters / 0.0254;
    }
};


// 速度类 - 内部使用米每秒(m/s)作为基本单位
class Velocity
{
private:
    double value_mps;  // meters per second 米/秒

public:
    Velocity(double mps = 0) : value_mps(mps)
    {
    }

    static Velocity MPS(double mps)
    {
        return Velocity(mps);
    }
    static Velocity FPS(double fps)  // feet per second 英尺/秒
    {
        return Velocity(fps * 0.3048);
    }

    double MPS() const
    {
        return value_mps;
    }
    double FPS() const
    {
        return value_mps / 0.3048;
    }
};


#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

// 角度类 - 内部使用弧度(radians)作为基本单位
class Angular
{
private:
    double value_radians;

    public:
    Angular(double radians = 0) : value_radians(radians)
    {
    }

    // 拷贝构造函数
    Angular(const Angular& other) = default;

    // 移动构造函数
    Angular(Angular&& other) noexcept = default;

    // 拷贝赋值运算符
    Angular& operator=(const Angular& other)
    {
        if (this != &other)
        {
            value_radians = other.value_radians;
        }
        return *this;
    }

    // 移动赋值运算符
    Angular& operator=(Angular&& other) noexcept
    {
        if (this != &other)
        {
            value_radians = other.value_radians;
        }
        return *this;
    }

    // 从double的赋值运算符
    Angular& operator=(double radians)
    {
        value_radians = radians;
        return *this;
    }

    //MOA
    static Angular MOA(double moa)  // 分
    {
        return Angular(moa * M_PI / (180.0 * 60.0));
    }

    //Mil
    static Angular Mil(double mils)  // 密位
    {
        return Angular(mils * M_PI / 3200.0);
    }

    static Angular Radians(double radians)
    {
        return Angular(radians);
    }

    static Angular Degrees(double degrees)  // 度
    {
        return Angular(degrees * M_PI / 180.0);
    }

    double toMil() const
    {
        return value_radians * 3200.0 / M_PI;
    }

    double toRadians() const
    {
        return value_radians;
    }
    double toDegrees() const
    {
        return value_radians * 180.0 / M_PI;
    }

    double sin() const
    {
        return std::sin(value_radians);
    }
    double cos() const
    {
        return std::cos(value_radians);
    }
};


// 温度
class Temperature
{
private:
    double value_celsius;  // 摄氏度

public:
    Temperature(double celsius = 0) : value_celsius(celsius)
    {
    }

    static Temperature Celsius(double celsius)   
    {
        return Temperature(celsius);
    }
    static Temperature Fahrenheit(double fahrenheit)  // 华氏度
    {
        return Temperature((fahrenheit - 32) * 5.0 / 9.0);
    }

    double toCelsius() const
    {
        return value_celsius;
    }
    double toFahrenheit() const
    {
        return value_celsius * 9.0 / 5.0 + 32;
    }
};


// 气压
class Pressure
{
private:
    double value_hpa;  // 百帕斯卡

public:
    Pressure(double hpa = 0) : value_hpa(hpa)
    {
    }

    static Pressure hPa(double hpa)
    {
        return Pressure(hpa);
    }
    static Pressure InHg(double inhg)  // 英寸汞柱
    {
        return Pressure(inhg * 33.8638865);
    }

    double tohPa() const
    {
        return value_hpa;
    }
    double toInHg() const
    {
        return value_hpa / 33.8638865;
    }
};


// 重量类 - 内部使用克(g)作为基本单位
class Weight
{
private:
    double value_grams;  // 内部存储单位为克

public:
    // 构造函数
    Weight(double grams = 0) : value_grams(grams)
    {
    }

    // 静态工厂方法
    static Weight Grams(double grams)
    {
        return Weight(grams);
    }

    static Weight Grains(double grains)
    {
        return Weight(grains / 15.432358353);  // 格令转克
    }

    static Weight Kilograms(double kg)
    {
        return Weight(kg * 1000.0);  // 千克转克
    }

    static Weight Pounds(double pounds)
    {
        return Weight(pounds * 453.592);  // 磅转克
    }

    static Weight Ounces(double ounces)
    {
        return Weight(ounces * 28.3495);  // 盎司转克
    }

    // 转换方法
    double toGrams() const
    {
        return value_grams;
    }

    double toGrains() const
    {
        return value_grams * 15.432358353;  // 克转格令
    }

    double toKilograms() const
    {
        return value_grams / 1000.0;
    }

    double toPounds() const
    {
        return value_grams / 453.592;
    }

    double toOunces() const
    {
        return value_grams / 28.3495;
    }

    // 算术运算符重载
    Weight operator+(const Weight& other) const
    {
        return Weight(value_grams + other.value_grams);
    }

    Weight operator-(const Weight& other) const
    {
        return Weight(value_grams - other.value_grams);
    }

    Weight operator*(double scalar) const
    {
        return Weight(value_grams * scalar);
    }

    Weight operator/(double scalar) const
    {
        return Weight(value_grams / scalar);
    }

    // 比较运算符
    bool operator==(const Weight& other) const
    {
        return value_grams == other.value_grams;
    }

    bool operator<(const Weight& other) const
    {
        return value_grams < other.value_grams;
    }

    bool operator>(const Weight& other) const
    {
        return value_grams > other.value_grams;
    }

    // 流输出运算符
    friend std::ostream& operator<<(std::ostream& os, const Weight& w)
    {
        os << w.value_grams << " g";
        return os;
    }
};

struct Vector3D
{
    double x, y, z;

    Vector3D(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z)
    {
    }

    double magnitude() const
    {
        return std::sqrt(x * x + y * y + z * z);
    }

    Vector3D operator*(double scalar) const
    {
        return Vector3D(x * scalar, y * scalar, z * scalar);
    }

    Vector3D operator+(const Vector3D& other) const
    {
        return Vector3D(x + other.x, y + other.y, z + other.z);
    }

    Vector3D operator-(const Vector3D& other) const
    {
        return Vector3D(x - other.x, y - other.y, z - other.z);
    }

};


#endif  // BALLISTIC_CALC_IMPL_UNIT_HPP