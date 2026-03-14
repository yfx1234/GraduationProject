// 解释：引入当前实现文件对应的头文件 `KalmanPredictor.h`，使实现部分能够看到类和函数声明。
#include "KalmanPredictor.h"

// ============================================================================
//  构造 / 初始化 / 重置
// ============================================================================

// 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
UKalmanPredictor::UKalmanPredictor()
    // 解释：这一行位于构造函数初始化列表中，把 `QScale` 直接初始化为 `1.0f`，减少进入函数体后的额外赋值开销。
    : QScale(1.0f)
    // 解释：这一行位于构造函数初始化列表中，把 `RScale` 直接初始化为 `0.5f`，减少进入函数体后的额外赋值开销。
    , RScale(0.5f)
    // 解释：这一行位于构造函数初始化列表中，把 `AdaptiveQ` 直接初始化为 `1.0f`，减少进入函数体后的额外赋值开销。
    , AdaptiveQ(1.0f)
    // 解释：这一行位于构造函数初始化列表中，把 `bInitialized` 直接初始化为 `false`，减少进入函数体后的额外赋值开销。
    , bInitialized(false)
    // 解释：这一行位于构造函数初始化列表中，把 `UpdateCount` 直接初始化为 `0`，减少进入函数体后的额外赋值开销。
    , UpdateCount(0)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行用 `FMemory::Memzero` 把指定内存块清零，常用于初始化矩阵、数组或状态缓存。
    FMemory::Memzero(X, sizeof(X));
    // 解释：这一行用 `FMemory::Memzero` 把指定内存块清零，常用于初始化矩阵、数组或状态缓存。
    FMemory::Memzero(P, sizeof(P));
    // 解释：这一行用 `FMemory::Memzero` 把指定内存块清零，常用于初始化矩阵、数组或状态缓存。
    FMemory::Memzero(ResidualVariance, sizeof(ResidualVariance));
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

// 解释：这一行定义函数 `Initialize`，开始实现initialize的具体逻辑。
void UKalmanPredictor::Initialize(float ProcessNoise, float MeasurementNoise)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行把右侧表达式的结果写入 `QScale`，完成 过程噪声缩放系数 的更新。
    QScale = ProcessNoise;
    // 解释：这一行把右侧表达式的结果写入 `RScale`，完成 观测噪声缩放系数 的更新。
    RScale = MeasurementNoise;
    // 解释：调用 `Reset` 执行当前步骤需要的功能逻辑。
    Reset();
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

// 解释：这一行定义函数 `Reset`，开始实现reset的具体逻辑。
void UKalmanPredictor::Reset()
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行用 `FMemory::Memzero` 把指定内存块清零，常用于初始化矩阵、数组或状态缓存。
    FMemory::Memzero(X, sizeof(X));
    // 解释：这一行用 `FMemory::Memzero` 把指定内存块清零，常用于初始化矩阵、数组或状态缓存。
    FMemory::Memzero(P, sizeof(P));
    // 初始协方差：位置 1000, 速度 100, 加速度 10
    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
    for (int32 i = 0; i < 3; i++) P[i][i]     = 1000.0;  // 位置不确定性
    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
    for (int32 i = 3; i < 6; i++) P[i][i]     = 100.0;   // 速度不确定性
    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
    for (int32 i = 6; i < 9; i++) P[i][i]     = 10.0;    // 加速度不确定性

    // 解释：这一行根据残差统计结果调整过程噪声大小。
    AdaptiveQ = (double)QScale;
    // 解释：这一行用 `FMemory::Memzero` 把指定内存块清零，常用于初始化矩阵、数组或状态缓存。
    FMemory::Memzero(ResidualVariance, sizeof(ResidualVariance));

    // 解释：这一行把右侧表达式的结果写入 `bInitialized`，完成 布尔标志 initialized 的更新。
    bInitialized = false;
    // 解释：这一行把右侧表达式的结果写入 `UpdateCount`，完成 更新计数器 的更新。
    UpdateCount = 0;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

// ============================================================================
//  核心：预测 + 更新  (9维 CA 模型)
// ============================================================================
// 状态向量 x = [px, py, pz, vx, vy, vz, ax, ay, az]
// 状态转移:
//   F = | I   dt*I   0.5*dt²*I |
//       | 0     I      dt*I    |
//       | 0     0        I     |
// 观测矩阵 H = [I  0  0]  (3×9)
// 过程噪声 Q 基于加加速度(jerk)驱动的连续白噪声离散化
// ============================================================================

// 解释：这一行定义函数 `Update`，开始实现update的具体逻辑。
void UKalmanPredictor::Update(const FVector& ObservedPos, float DeltaTime)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (DeltaTime <= 0.0f) return;

    // ---------- 首次观测：直接初始化 ----------
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!bInitialized)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `X[0]`，完成 状态向量 X 的更新。
        X[0] = ObservedPos.X;   // px
        // 解释：这一行把右侧表达式的结果写入 `X[1]`，完成 状态向量 X 的更新。
        X[1] = ObservedPos.Y;   // py
        // 解释：这一行把右侧表达式的结果写入 `X[2]`，完成 状态向量 X 的更新。
        X[2] = ObservedPos.Z;   // pz
        // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
        for (int32 i = 3; i < N_STATE; i++) X[i] = 0.0;
        // 解释：这一行把右侧表达式的结果写入 `bInitialized`，完成 布尔标志 initialized 的更新。
        bInitialized = true;
        // 解释：这一行把右侧表达式的结果写入 `UpdateCount`，完成 更新计数器 的更新。
        UpdateCount = 1;
        // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
        return;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // 解释：这一行把右侧表达式的结果写入 `const double dt`，完成 constdoubledt 的更新。
    const double dt  = (double)DeltaTime;
    // 解释：这一行声明成员或局部变量 `dt2`，用于保存dt2。
    const double dt2 = dt * dt;
    // 解释：这一行声明成员或局部变量 `dt3`，用于保存dt3。
    const double dt3 = dt2 * dt;
    // 解释：这一行声明成员或局部变量 `dt4`，用于保存dt4。
    const double dt4 = dt3 * dt;
    // 解释：这一行声明成员或局部变量 `dt5`，用于保存dt5。
    const double dt5 = dt4 * dt;

    // ======== 1) 构建状态转移矩阵 F (9×9) ========
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    double F[N_STATE * N_STATE];
    // 解释：调用 `MatIdentity` 执行当前步骤需要的功能逻辑。
    MatIdentity(F, N_STATE);
    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
    for (int32 i = 0; i < 3; i++)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `F[i * N_STATE + (i + 3)]`，完成 状态转移矩阵 F 的更新。
        F[i * N_STATE + (i + 3)] = dt;            // pos += vel*dt
        // 解释：这一行把右侧表达式的结果写入 `F[i * N_STATE + (i + 6)]`，完成 状态转移矩阵 F 的更新。
        F[i * N_STATE + (i + 6)] = 0.5 * dt2;     // pos += 0.5*acc*dt²
        // 解释：这一行把右侧表达式的结果写入 `F[(i + 3) * N_STATE + (i + 6)]`，完成 状态转移矩阵 F 的更新。
        F[(i + 3) * N_STATE + (i + 6)] = dt;      // vel += acc*dt
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // ======== 2) 先验状态预测: XPred = F * X ========
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    double XPred[N_STATE];
    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
    for (int32 i = 0; i < N_STATE; i++)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行计算先验状态预测结果。
        XPred[i] = 0.0;
        // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
        for (int32 j = 0; j < N_STATE; j++)
            // 解释：这一行计算先验状态预测结果。
            XPred[i] += F[i * N_STATE + j] * X[j];
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // ======== 3) 先验协方差: PPred = F*P*F^T + Q ========
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    double FT[N_STATE * N_STATE];
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    double FP[N_STATE * N_STATE];
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    double PPred[N_STATE * N_STATE];
    // 解释：调用 `MatTranspose` 执行当前步骤需要的功能逻辑。
    MatTranspose(F, FT, N_STATE);
    // 解释：调用 `MatMul` 执行当前步骤需要的功能逻辑。
    MatMul(F, (const double*)P, FP, N_STATE);
    // 解释：调用 `MatMul` 执行当前步骤需要的功能逻辑。
    MatMul(FP, FT, PPred, N_STATE);

    // 构建过程噪声 Q (基于 jerk 驱动离散化，参考 Bar-Shalom)
    // 对每个轴 i, Q 的 3×3 块 (pos_i, vel_i, acc_i) 为:
    //   q * | dt^5/20  dt^4/8  dt^3/6 |
    //       | dt^4/8   dt^3/3  dt^2/2 |
    //       | dt^3/6   dt^2/2    dt   |
    // 解释：这一行声明成员或局部变量 `q`，用于保存Q。
    const double q = AdaptiveQ;
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    double Q[N_STATE * N_STATE];
    // 解释：这一行用 `FMemory::Memzero` 把指定内存块清零，常用于初始化矩阵、数组或状态缓存。
    FMemory::Memzero(Q, sizeof(Q));
    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
    for (int32 axis = 0; axis < 3; axis++)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行声明成员或局部变量 `p`，用于保存P。
        const int32 p = axis;       // 位置索引
        // 解释：这一行声明成员或局部变量 `v`，用于保存V。
        const int32 v = axis + 3;   // 速度索引
        // 解释：这一行声明成员或局部变量 `a`，用于保存A。
        const int32 a = axis + 6;   // 加速度索引

        // 解释：这一行把右侧表达式的结果写入 `Q[p * N_STATE + p]`，完成 过程噪声矩阵 Q 的更新。
        Q[p * N_STATE + p] = q * dt5 / 20.0;
        // 解释：这一行把右侧表达式的结果写入 `Q[p * N_STATE + v]`，完成 过程噪声矩阵 Q 的更新。
        Q[p * N_STATE + v] = q * dt4 / 8.0;
        // 解释：这一行把右侧表达式的结果写入 `Q[v * N_STATE + p]`，完成 过程噪声矩阵 Q 的更新。
        Q[v * N_STATE + p] = q * dt4 / 8.0;
        // 解释：这一行把右侧表达式的结果写入 `Q[p * N_STATE + a]`，完成 过程噪声矩阵 Q 的更新。
        Q[p * N_STATE + a] = q * dt3 / 6.0;
        // 解释：这一行把右侧表达式的结果写入 `Q[a * N_STATE + p]`，完成 过程噪声矩阵 Q 的更新。
        Q[a * N_STATE + p] = q * dt3 / 6.0;
        // 解释：这一行把右侧表达式的结果写入 `Q[v * N_STATE + v]`，完成 过程噪声矩阵 Q 的更新。
        Q[v * N_STATE + v] = q * dt3 / 3.0;
        // 解释：这一行把右侧表达式的结果写入 `Q[v * N_STATE + a]`，完成 过程噪声矩阵 Q 的更新。
        Q[v * N_STATE + a] = q * dt2 / 2.0;
        // 解释：这一行把右侧表达式的结果写入 `Q[a * N_STATE + v]`，完成 过程噪声矩阵 Q 的更新。
        Q[a * N_STATE + v] = q * dt2 / 2.0;
        // 解释：这一行把右侧表达式的结果写入 `Q[a * N_STATE + a]`，完成 过程噪声矩阵 Q 的更新。
        Q[a * N_STATE + a] = q * dt;
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 解释：调用 `MatAdd` 执行当前步骤需要的功能逻辑。
    MatAdd(PPred, Q, PPred, N_STATE);

    // ======== 4) 计算残差 y = z - H*XPred ========
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    double y[N_OBS];
    // 解释：这一行把右侧表达式的结果写入 `y[0]`，完成 观测残差 y 的更新。
    y[0] = ObservedPos.X - XPred[0];
    // 解释：这一行把右侧表达式的结果写入 `y[1]`，完成 观测残差 y 的更新。
    y[1] = ObservedPos.Y - XPred[1];
    // 解释：这一行把右侧表达式的结果写入 `y[2]`，完成 观测残差 y 的更新。
    y[2] = ObservedPos.Z - XPred[2];

    // ======== 5) 残差协方差 S = H*PPred*H^T + R ========
    // H = [I 0 0], 所以 S = PPred[0:3,0:3] + R*I
    // 解释：这一行把右侧表达式的结果写入 `const double r`，完成 constdoubleR 的更新。
    const double r = (double)RScale;
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    double S[3][3];
    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
    for (int32 i = 0; i < N_OBS; i++)
        // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
        for (int32 j = 0; j < N_OBS; j++)
            // 解释：这一行把右侧表达式的结果写入 `S[i][j]`，完成 残差协方差矩阵 S 的更新。
            S[i][j] = PPred[i * N_STATE + j] + (i == j ? r : 0.0);

    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    double SInv[3][3];
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!MatInverse3(S, SInv)) return;

    // ======== 6) 卡尔曼增益 K = PPred * H^T * S^-1  (9×3) ========
    // K[i][j] = sum_k PPred[i][k] * H^T[k][j] * SInv  但 H^T 只在 k<3 时非零
    // 所以 K[i][j] = sum_k PPred[i][k] * SInv[k][j], k=0..2
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    double K[N_STATE][N_OBS];
    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
    for (int32 i = 0; i < N_STATE; i++)
        // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
        for (int32 j = 0; j < N_OBS; j++)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行把右侧表达式的结果写入 `K[i][j]`，完成 卡尔曼增益矩阵 K 的更新。
            K[i][j] = 0.0;
            // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
            for (int32 k = 0; k < N_OBS; k++)
                // 解释：这一行在 `K[i][j]` 的原有基础上继续累加新量，用于持续更新 卡尔曼增益矩阵 K。
                K[i][j] += PPred[i * N_STATE + k] * SInv[k][j];
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }

    // ======== 7) 后验状态: X = XPred + K * y ========
    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
    for (int32 i = 0; i < N_STATE; i++)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行把右侧表达式的结果写入 `X[i]`，完成 状态向量 X 的更新。
        X[i] = XPred[i];
        // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
        for (int32 j = 0; j < N_OBS; j++)
            // 解释：这一行在 `X[i]` 的原有基础上继续累加新量，用于持续更新 状态向量 X。
            X[i] += K[i][j] * y[j];
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }

    // ======== 8) 后验协方差: P = (I - K*H) * PPred ========
    // KH (9×9) = K * H, 其中 H=[I 0 0], 所以 KH[i][j] = K[i][j] if j<3, else 0
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    double IMinusKH[N_STATE * N_STATE];
    // 解释：调用 `MatIdentity` 执行当前步骤需要的功能逻辑。
    MatIdentity(IMinusKH, N_STATE);
    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
    for (int32 i = 0; i < N_STATE; i++)
        // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
        for (int32 j = 0; j < N_OBS; j++)
            // 解释：这一行从 `IMinusKH[i * N_STATE + j]` 中减去新量，用于修正或抵消 iminuskh。
            IMinusKH[i * N_STATE + j] -= K[i][j];

    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    double PNew[N_STATE * N_STATE];
    // 解释：调用 `MatMul` 执行当前步骤需要的功能逻辑。
    MatMul(IMinusKH, PPred, PNew, N_STATE);
    // 解释：这一行用 `FMemory::Memcpy` 把临时计算结果整体复制回目标缓冲区。
    FMemory::Memcpy(P, PNew, sizeof(P));

    // ======== 9) 自适应过程噪声 (EWMA 残差方差估计) ========
    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
    for (int32 i = 0; i < N_OBS; i++)
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    {
        // 解释：这一行声明成员或局部变量 `residSq`，用于保存residsq。
        const double residSq = y[i] * y[i];
        // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
        if (UpdateCount <= 2)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行把右侧表达式的结果写入 `ResidualVariance[i]`，完成 残差方差估计 的更新。
            ResidualVariance[i] = residSq;
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
        // 解释：这一行给出兜底分支，当上面的条件都不满足时执行。
        else
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
            ResidualVariance[i] = AdaptiveAlpha * residSq
                                // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                                + (1.0 - AdaptiveAlpha) * ResidualVariance[i];
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
    }
    // 计算新 Q：若残差方差远高于 R，说明目标在机动, 需要增大 Q
    // 解释：这一行把右侧表达式的结果写入 `const double avgResidVar`，完成 constdoubleavgresidvar 的更新。
    const double avgResidVar = (ResidualVariance[0] + ResidualVariance[1] + ResidualVariance[2]) / 3.0;
    // 解释：这一行通过 `FMath::Max` 给 `const double ratio` 施加下界约束，避免 constdoubleratio 过小。
    const double ratio = avgResidVar / FMath::Max(r, 0.001);
    // 解释：这一行根据残差统计结果调整过程噪声大小。
    AdaptiveQ = FMath::Clamp(ratio * (double)QScale, AdaptiveQMin, AdaptiveQMax);

    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    UpdateCount++;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

// ============================================================================
//  预测 / 输出接口
// ============================================================================

// 解释：这一行定义函数 `PredictPosition`，开始实现predictposition的具体逻辑。
FVector UKalmanPredictor::PredictPosition(float dt) const
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (!bInitialized) return FVector::ZeroVector;
    // 解释：这一行把右侧表达式的结果写入 `const double t`，完成 constdoubleT 的更新。
    const double t  = (double)dt;
    // 解释：这一行声明成员或局部变量 `t2`，用于保存t2。
    const double t2 = t * t;
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return FVector(
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        X[0] + X[3] * t + 0.5 * X[6] * t2,   // px + vx*t + 0.5*ax*t²
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        X[1] + X[4] * t + 0.5 * X[7] * t2,   // py + vy*t + 0.5*ay*t²
        // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
        X[2] + X[5] * t + 0.5 * X[8] * t2    // pz + vz*t + 0.5*az*t²
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    );
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

// 解释：这一行定义函数 `GetEstimatedVelocity`，开始实现getestimatedvelocity的具体逻辑。
FVector UKalmanPredictor::GetEstimatedVelocity() const
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return FVector(X[3], X[4], X[5]);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

// 解释：这一行定义函数 `GetEstimatedPosition`，开始实现getestimatedposition的具体逻辑。
FVector UKalmanPredictor::GetEstimatedPosition() const
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return FVector(X[0], X[1], X[2]);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

// 解释：这一行定义函数 `GetEstimatedAcceleration`，开始实现getestimatedacceleration的具体逻辑。
FVector UKalmanPredictor::GetEstimatedAcceleration() const
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return FVector(X[6], X[7], X[8]);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

// 解释：这一行定义函数 `GetPositionUncertainty`，开始实现getpositionuncertainty的具体逻辑。
float UKalmanPredictor::GetPositionUncertainty() const
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return (float)(P[0][0] + P[1][1] + P[2][2]);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

// ============================================================================
//  N×N 通用矩阵工具函数
// ============================================================================

// 解释：这一行定义函数 `MatMul`，开始实现matmul的具体逻辑。
void UKalmanPredictor::MatMul(const double* A, const double* B, double* C, int32 Dim)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 使用临时缓冲区支持就地运算 (C == A 或 C == B)
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    double Temp[N_STATE * N_STATE];
    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
    for (int32 i = 0; i < Dim; i++)
        // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
        for (int32 j = 0; j < Dim; j++)
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        {
            // 解释：这一行把右侧表达式的结果写入 `Temp[i * Dim + j]`，完成 temp 的更新。
            Temp[i * Dim + j] = 0.0;
            // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
            for (int32 k = 0; k < Dim; k++)
                // 解释：这一行在 `Temp[i * Dim + j]` 的原有基础上继续累加新量，用于持续更新 temp。
                Temp[i * Dim + j] += A[i * Dim + k] * B[k * Dim + j];
        // 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
        }
    // 解释：这一行用 `FMemory::Memcpy` 把临时计算结果整体复制回目标缓冲区。
    FMemory::Memcpy(C, Temp, sizeof(double) * Dim * Dim);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

// 解释：这一行定义函数 `MatTranspose`，开始实现mattranspose的具体逻辑。
void UKalmanPredictor::MatTranspose(const double* A, double* AT, int32 Dim)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    double Temp[N_STATE * N_STATE];
    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
    for (int32 i = 0; i < Dim; i++)
        // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
        for (int32 j = 0; j < Dim; j++)
            // 解释：这一行把右侧表达式的结果写入 `Temp[j * Dim + i]`，完成 temp 的更新。
            Temp[j * Dim + i] = A[i * Dim + j];
    // 解释：这一行用 `FMemory::Memcpy` 把临时计算结果整体复制回目标缓冲区。
    FMemory::Memcpy(AT, Temp, sizeof(double) * Dim * Dim);
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

// 解释：这一行定义函数 `MatAdd`，开始实现matadd的具体逻辑。
void UKalmanPredictor::MatAdd(const double* A, const double* B, double* C, int32 Dim)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行声明成员或局部变量 `N`，用于保存N。
    const int32 N = Dim * Dim;
    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
    for (int32 i = 0; i < N; i++)
        // 解释：这一行把右侧表达式的结果写入 `C[i]`，完成 C 的更新。
        C[i] = A[i] + B[i];
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

// 解释：这一行定义函数 `MatSub`，开始实现matsub的具体逻辑。
void UKalmanPredictor::MatSub(const double* A, const double* B, double* C, int32 Dim)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行声明成员或局部变量 `N`，用于保存N。
    const int32 N = Dim * Dim;
    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
    for (int32 i = 0; i < N; i++)
        // 解释：这一行把右侧表达式的结果写入 `C[i]`，完成 C 的更新。
        C[i] = A[i] - B[i];
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

// 解释：这一行定义函数 `MatIdentity`，开始实现matidentity的具体逻辑。
void UKalmanPredictor::MatIdentity(double* A, int32 Dim)
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行用 `FMemory::Memzero` 把指定内存块清零，常用于初始化矩阵、数组或状态缓存。
    FMemory::Memzero(A, sizeof(double) * Dim * Dim);
    // 解释：这一行开始 `for` 循环，用于按既定次数或序列遍历执行后续逻辑。
    for (int32 i = 0; i < Dim; i++)
        // 解释：这一行把右侧表达式的结果写入 `A[i * Dim + i]`，完成 A 的更新。
        A[i * Dim + i] = 1.0;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}

// 解释：这一行定义函数 `MatInverse3`，开始实现matinverse3的具体逻辑。
bool UKalmanPredictor::MatInverse3(const double A[3][3], double Ainv[3][3])
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
{
    // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
    const double det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
                     // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                     - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
                     // 解释：这一行落实当前模块中的具体实现细节，为上面的声明、公式或控制流程提供实际执行语句。
                     + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
    // 解释：这一行给出 `if` 条件判断，只有条件成立时才会进入下面的分支逻辑。
    if (FMath::Abs(det) < 1e-12) return false;

    // 解释：这一行声明成员或局部变量 `invDet`，用于保存invdet。
    const double invDet = 1.0 / det;
    // 解释：这一行把右侧表达式的结果写入 `Ainv[0][0]`，完成 ainv 的更新。
    Ainv[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) * invDet;
    // 解释：这一行把右侧表达式的结果写入 `Ainv[0][1]`，完成 ainv 的更新。
    Ainv[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) * invDet;
    // 解释：这一行把右侧表达式的结果写入 `Ainv[0][2]`，完成 ainv 的更新。
    Ainv[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * invDet;
    // 解释：这一行把右侧表达式的结果写入 `Ainv[1][0]`，完成 ainv 的更新。
    Ainv[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) * invDet;
    // 解释：这一行把右侧表达式的结果写入 `Ainv[1][1]`，完成 ainv 的更新。
    Ainv[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * invDet;
    // 解释：这一行把右侧表达式的结果写入 `Ainv[1][2]`，完成 ainv 的更新。
    Ainv[1][2] = (A[0][2] * A[1][0] - A[0][0] * A[1][2]) * invDet;
    // 解释：这一行把右侧表达式的结果写入 `Ainv[2][0]`，完成 ainv 的更新。
    Ainv[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) * invDet;
    // 解释：这一行把右侧表达式的结果写入 `Ainv[2][1]`，完成 ainv 的更新。
    Ainv[2][1] = (A[0][1] * A[2][0] - A[0][0] * A[2][1]) * invDet;
    // 解释：这一行把右侧表达式的结果写入 `Ainv[2][2]`，完成 ainv 的更新。
    Ainv[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * invDet;
    // 解释：这一行返回当前函数的计算结果，把控制权交回调用方。
    return true;
// 解释：这一行用于开始或结束当前作用域，控制类、函数或条件块的边界。
}
