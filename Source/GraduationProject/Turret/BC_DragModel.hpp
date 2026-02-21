#ifndef BALLISTIC_CALC_IMPL_DRAG_MODEL_HPP
#define BALLISTIC_CALC_IMPL_DRAG_MODEL_HPP

#include "BC_Unit.hpp"

#include <vector>

#include "BC_DragTables.hpp"

// 符号判断函数
inline int sign(double value)
{
    if (value > 0)
        return 1;
    if (value < 0)
        return -1;
    return 0;
}

// 确保输入序列严格递增
inline void ensure_strictly_increasing(const std::vector<double>& xs)
{
    for (size_t i = 1; i < xs.size(); ++i)
    {
        if (xs[i] <= xs[i - 1])
        {
            throw std::invalid_argument("x值必须严格递增且无重复");
        }
    }
}

class Pchip
{
public:
    std::vector<double> xs;  // 节点位置，严格递增
    std::vector<double> ys;  // 
    std::vector<double> a;  // 常数项系数
    std::vector<double> b;  // 一次项系数
    std::vector<double> c;  // 二次项系数
    std::vector<double> d;  // 三次项系数

    Pchip() {};


    void init(const std::vector<double>& _xs, const std::vector<double>& _ys)
    {
        xs = _xs;
        ys = _ys;

        // 输入验证
        if (xs.size() != ys.size())
        {
            throw std::invalid_argument("xs和ys的长度必须相同");
        }

        size_t n = xs.size();
        if (n < 2)
        {
            throw std::invalid_argument("插值至少需要两个点");
        }

        ensure_strictly_increasing(xs);

        // 计算步长h和有限差分delta
        std::vector<double> h(n - 1);
        std::vector<double> delta(n - 1);

        for (size_t i = 0; i < n - 1; ++i)
        {
            h[i] = xs[i + 1] - xs[i];
            delta[i] = (ys[i + 1] - ys[i]) / h[i];
        }

        // 使用Fritsch-Carlson方法计算节点处的斜率m
        std::vector<double> m(n, 0.0);

        if (n == 2)
        {
            // 线性段：两端斜率都等于delta
            m[0] = delta[0];
            m[1] = delta[0];
        }
        else
        {
            // 内部节点处理
            for (size_t i = 1; i < n - 1; ++i)
            {
                double d0 = delta[i - 1];
                double d1 = delta[i];

                if (d0 == 0.0 || d1 == 0.0 || sign(d0) != sign(d1))
                {
                    m[i] = 0.0;
                }
                else
                {
                    double w1 = 2 * h[i] + h[i - 1];
                    double w2 = h[i] + 2 * h[i - 1];
                    m[i] = (w1 + w2) / (w1 / d0 + w2 / d1);
                }
            }

            // 左端点处理（三点公式+限制）
            double d0 = delta[0];
            double d1 = delta[1];
            double m0 = ((2 * h[0] + h[1]) * d0 - h[0] * d1) / (h[0] + h[1]);

            if (sign(m0) != sign(d0))
            {
                m0 = 0.0;
            }
            else if (std::abs(m0) > 3 * std::abs(d0))
            {
                m0 = 3 * d0;
            }
            m[0] = m0;

            // 右端点处理
            double dn_2 = delta[n - 2];
            double dn_3 = delta[n - 3];
            double mn = ((2 * h[n - 2] + h[n - 3]) * dn_2 - h[n - 2] * dn_3) / (h[n - 2] + h[n - 3]);

            if (sign(mn) != sign(dn_2))
            {
                mn = 0.0;
            }
            else if (std::abs(mn) > 3 * std::abs(dn_2))
            {
                mn = 3 * dn_2;
            }
            m[n - 1] = mn;
        }

        // 将Hermite形式转换为非归一化多项式系数
        a.resize(n - 1);
        b.resize(n - 1);
        c.resize(n - 1);
        d.resize(n - 1);

        for (size_t i = 0; i < n - 1; ++i)
        {
            double y0 = ys[i];
            double y1 = ys[i + 1];
            double h_i = h[i];
            double m0 = m[i];
            double m1 = m[i + 1];

            a[i] = y0;
            b[i] = m0;
            c[i] = (3 * (y1 - y0) - (2 * m0 + m1) * h_i) / (h_i * h_i);
            d[i] = (2 * (y0 - y1) + (m0 + m1) * h_i) / (h_i * h_i * h_i);
        }
    }

    double eval(double x) const
    {
        size_t n = xs.size();

        // 边界检查
        if (x <= xs[0])
        {
            return a[0];  // 左外推
        }
        if (x >= xs[n - 1])
        {
            return a[n - 2] +
                (x - xs[n - 2]) *
                (b[n - 2] + (x - xs[n - 2]) * (c[n - 2] + (x - xs[n - 2]) * d[n - 2]));  // 右外推
        }

        // 二分查找定位区间
        size_t lo = 0, hi = n - 1;
        while (hi - lo > 1)
        {
            size_t mid = lo + (hi - lo) / 2;
            if (xs[mid] <= x)
            {
                lo = mid;
            }
            else
            {
                hi = mid;
            }
        }

        // 使用Horner法则高效计算三次多项式
        size_t i = lo;
        double dx = x - xs[i];
        return a[i] + dx * (b[i] + dx * (c[i] + dx * d[i]));
    }
};

class DragModel
{
public:
    double bc;                             // 阻力系数
    std::vector<DragDataPoint> dragTable;  // 阻力表
    Weight weight;                         // 弹丸重量
    Distance diameter;                     // 弹丸直径
    Distance length;                       // 弹丸长度
    Pchip pchip;

private:
    float sectional_density = 0.0f;  // 截面密度
    float form_factor = 0.0f;        // 弹型系数

public:
    DragModel(double bc, const std::vector<DragDataPoint>& table, Weight w = Weight(), Distance d = Distance(),
        Distance l = Distance())
        : bc(bc), dragTable(table), weight(w), diameter(d), length(l)
    {

        if (bc <= 0)
        {
            throw std::invalid_argument("Ballistic coefficient must be positive");
        }

        if (table.empty())
        {
            throw std::invalid_argument("Received empty drag table");
        }

        // 如果只有一个数据点，添加一个稍微不同的点以避免插值问题
        if (dragTable.size() < 2)
        {
            dragTable.push_back(DragDataPoint(dragTable[0].Mach + 0.1, dragTable[0].CD));
        }

        // 计算截面密度和弹型系数
        sectional_density = getSectionalDensity();
        form_factor = getFormFactor(bc);

        std::vector<double> xs;
        std::vector<double> ys;
        for (int i = 0; i < dragTable.size(); i++)
        {
            xs.push_back(dragTable[i].Mach);
            ys.push_back(dragTable[i].CD);
        }
        pchip.init(xs, ys);
    }

    // 获取弹型系数
    double getBC() const
    {
        return bc;
    }

    // 获取阻力系数，基于马赫数插值
    double getDragCoefficient(double mach) const
    {
        if (dragTable.empty())
            return 0.3;  // 默认值

        return pchip.eval(mach);  //使用插值的方式

        // 边界检查
        if (mach <= dragTable.front().Mach)
        {
            return dragTable.front().CD;
        }
        if (mach >= dragTable.back().Mach)
        {
            return dragTable.back().CD;
        }

        // 线性插值
        for (size_t i = 0; i < dragTable.size() - 1; i++)
        {
            if (mach >= dragTable[i].Mach && mach <= dragTable[i + 1].Mach)
            {
                double t = (mach - dragTable[i].Mach) / (dragTable[i + 1].Mach - dragTable[i].Mach);
                return dragTable[i].CD + t * (dragTable[i + 1].CD - dragTable[i].CD);
            }
        }

        return dragTable.back().CD;
    }

    // 【修改】参数名从 bc 改为 InBC 以避免遮挡成员变量
    const double getFormFactor(double InBC) const
    {
        if (InBC == 0)
            return 0.0;
        return getSectionalDensity() / InBC;
    }

    //获取截面密度 
    const double getSectionalDensity() const
    {
        if (diameter.Meters() == 0)
            return 0.0;

        // 采用磅/英寸^2
        return weight.toGrams() / (7000 * diameter.Inches() * diameter.Inches());
    }

    // 获取阻力表
    const std::vector<DragDataPoint>& getDragTable() const
    {
        return dragTable;
    }

    // cout 流输出
    friend std::ostream& operator<<(std::ostream& os, const DragModel& dm)
    {
        os << "DragModel(BC=" << dm.bc << ", Weight=" << dm.weight << " g, Diameter=" << dm.diameter.toMillimeters()
            << " mm, Length=" << dm.length.toMillimeters() << " mm)";
        return os;
    }
};

#endif