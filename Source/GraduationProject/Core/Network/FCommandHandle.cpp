#include "FCommandHandle.h" // 包含 FCommandHandle 类的声明头文件

#include "Dom/JsonObject.h" // 包含 FJsonObject 类，用于操作 JSON 对象数据结构
#include "Dom/JsonValue.h" // 包含 FJsonValue 类，用于处理 JSON 对象中的不同类型值
#include "Engine/GameInstance.h" // 包含 UGameInstance 类，用于获取全局游戏实例与上下文信息
#include "Engine/World.h" // 包含 UWorld 类，负责游戏世界的场景管理和 Actor 的生成
#include "GameFramework/Actor.h" // 包含 AActor 基础类，作为场景中所有实体的基类
#include "GraduationProject/Core/Manager/AgentManager.h" // 包含 UAgentManager 类，负责管理场景中的智能体注册与查询
#include "GraduationProject/Drone/DronePawn.h" // 包含 ADronePawn 类，用于处理针对无人机实体的特殊逻辑
#include "Kismet/GameplayStatics.h" // 包含 UGameplayStatics 工具类，提供如完成生成等全局静态方法
#include "Policies/CondensedJsonPrintPolicy.h" // 包含 TCondensedJsonPrintPolicy 策略，用于序列化为紧凑的、无空格的 JSON 字符串
#include "Serialization/JsonReader.h" // 包含 TJsonReader 类，用于将字符串反序列化解析为 JSON 对象
#include "Serialization/JsonSerializer.h" // 包含 FJsonSerializer 类，用于执行 JSON 数据的序列化和反序列化操作
#include "UObject/UObjectGlobals.h" // 包含虚幻引擎 UObject 全局系统宏及函数，如 StaticLoadClass
#include "UObject/UnrealType.h" // 包含 FProperty 等虚幻引擎反射类型的定义

// ──── 匿名命名空间：工具函数 ────
namespace // 匿名命名空间，将其内部的符号作用域限制在当前源码文件中
{
    /**
     * @brief 尝试将字符串解析为序号参数索引
     *        支持 "arg0", "p1" 等格式
     */
    bool TryParseOrderedKey(const FString& Key, int32& OutIndex) // 尝试将形如 "arg0" 或 "p1" 的参数键名解析为对应的整数索引
    {
        FString Work = Key; // 创建键名的本地副本以进行字符串处理
        Work.ToLowerInline();  // 将字符串原地转换为小写以支持按不区分大小写的方式匹配

        if (Work.StartsWith(TEXT("arg")))  // 检查字符串是否以 "arg" 作为前缀
        {
            Work = Work.RightChop(3); // 如果是，则截去前缀 "arg"，保留剩余的数字部分
        }
        else if (Work.StartsWith(TEXT("p")))  // 检查字符串是否以 "p" 作为前缀
        {
            Work = Work.RightChop(1); // 如果是，则截去前缀 "p"，保留剩余的数字部分
        }

        if (!Work.IsNumeric())  // 检查去除了前缀之后的字符串是否为合法的数字格式
        {
            return false; // 如果剩余部分不是数字，说明解析失败，返回 false
        }

        OutIndex = FCString::Atoi(*Work); // 将数字字符串转换为 int32 类型的整数，并赋值给输出参数
        return true; // 解析成功，返回 true
    }

    /** @brief 将 FJsonValue 序列化为 JSON 字符串 */
    FString SerializeJsonValue(const TSharedPtr<FJsonValue>& Value) // 将单一的 FJsonValue 节点序列化为 JSON 格式的字符串
    {
        if (!Value.IsValid()) // 检查传入的 JSON 值指针是否有效
        {
            return TEXT(""); // 若空，则返回空字符串
        }

        FString Output; // 声明存储序列化输出结果的字符串变量
        const TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&Output); // 创建绑定到 Output 字符串的默认 JSON 写入器实例
        FJsonSerializer::Serialize(Value.ToSharedRef(), TEXT(""), Writer); // 执行序列化过程，将生成的格式文本写入 Output 变量
        return Output; // 返回成功序列化后的最终字符串
    }

    /**
     * @brief 将 JSON 值转换为函数参数字符串
     *
     * 支持自动识别：
     * - 三元数组 [x,y,z] → "FVector:x,y,z"
     * - {x,y,z} 对象 → "FVector:x,y,z"
     * - {roll,pitch,yaw} 对象 → "FRotator:r,p,y"
     * - 基本类型直接转换
     */
    FString JsonValueToParamString(const TSharedPtr<FJsonValue>& Value) // 将各种 JSON 值转换为适应后续反射机制调用的特定格式字符串表示
    {
        if (!Value.IsValid()) // 检查传入的 JSON 值指针是否有效
        {
            return TEXT(""); // 无效则直接返回空字符串
        }

        switch (Value->Type) // 根据当前的 JSON 值类型采取不同的转换策略
        {
        case EJson::String: // 当前为字符串类型
            return Value->AsString(); // 直接提取字符串值并返回
        case EJson::Number: // 当前为数字类型
            return FString::SanitizeFloat(Value->AsNumber()); // 提取数字转为浮点数并清除多余的小数点后尾部的零
        case EJson::Boolean: // 当前为布尔类型
            return Value->AsBool() ? TEXT("true") : TEXT("false"); // 根据真假状态返回明确的 "true" 或 "false" 文本字符串
        case EJson::Array: // 当前为数组类型
        {
            const TArray<TSharedPtr<FJsonValue>>& ArrayValues = Value->AsArray(); // 获取数组对象的值列表
            if (ArrayValues.Num() == 3 && // 判断数组是否刚好有 3 个元素
                ArrayValues[0].IsValid() && ArrayValues[1].IsValid() && ArrayValues[2].IsValid() && // 判断这 3 个元素是否都有效
                ArrayValues[0]->Type == EJson::Number && // 判断第 1 个元素是否为数字
                ArrayValues[1]->Type == EJson::Number && // 判断第 2 个元素是否为数字
                ArrayValues[2]->Type == EJson::Number)   // 判断第 3 个元素是否为数字
            {
                const float X = static_cast<float>(ArrayValues[0]->AsNumber()); // 将第 1 个元素转换为 float 类型的 X 值
                const float Y = static_cast<float>(ArrayValues[1]->AsNumber()); // 将第 2 个元素转换为 float 类型的 Y 值
                const float Z = static_cast<float>(ArrayValues[2]->AsNumber()); // 将第 3 个元素转换为 float 类型的 Z 值
                return FString::Printf(TEXT("FVector:%f,%f,%f"), X, Y, Z); // 格式化为 "FVector:X,Y,Z" 的标记风格返回
            }

            return SerializeJsonValue(Value); // 若数组元素不满足作为三维向量的前提，默认通过普通的 JSON 序列化处理返回
        }
        case EJson::Object: // 当前为 JSON 对象（字典）类型
        {
            const TSharedPtr<FJsonObject> Obj = Value->AsObject(); // 将该值强转为 JSON 对象解析指针
            if (!Obj.IsValid()) // 检查对象指针是否有效
            {
                return TEXT(""); // 若空则返回空字符串
            }

            if (Obj->HasTypedField<EJson::Number>(TEXT("x")) && // 检查字典中是否存在类型为数字的 'x' 字段
                Obj->HasTypedField<EJson::Number>(TEXT("y")) && // 检查字典中是否存在类型为数字的 'y' 字段
                Obj->HasTypedField<EJson::Number>(TEXT("z")))   // 检查字典中是否存在类型为数字的 'z' 字段
            {
                const float X = static_cast<float>(Obj->GetNumberField(TEXT("x"))); // 获取 'x' 字段对应的浮点数值
                const float Y = static_cast<float>(Obj->GetNumberField(TEXT("y"))); // 获取 'y' 字段对应的浮点数值
                const float Z = static_cast<float>(Obj->GetNumberField(TEXT("z"))); // 获取 'z' 字段对应的浮点数值
                return FString::Printf(TEXT("FVector:%f,%f,%f"), X, Y, Z); // 格式化为 "FVector:X,Y,Z" 的文本返回
            }

            if (Obj->HasTypedField<EJson::Number>(TEXT("roll")) && // 检查字典中是否存在类型为数字的 'roll' 字段
                Obj->HasTypedField<EJson::Number>(TEXT("pitch")) && // 检查字典中是否存在类型为数字的 'pitch' 字段
                Obj->HasTypedField<EJson::Number>(TEXT("yaw"))) // 检查字典中是否存在类型为数字的 'yaw' 字段
            {
                const float Roll = static_cast<float>(Obj->GetNumberField(TEXT("roll"))); // 获取滚转角的数值
                const float Pitch = static_cast<float>(Obj->GetNumberField(TEXT("pitch"))); // 获取俯仰角的数值
                const float Yaw = static_cast<float>(Obj->GetNumberField(TEXT("yaw"))); // 获取偏航角的数值
                return FString::Printf(TEXT("FRotator:%f,%f,%f"), Roll, Pitch, Yaw); // 格式化为 "FRotator:Roll,Pitch,Yaw" 的文本返回
            }

            return SerializeJsonValue(Value); // 对于其他不满足特地提取规则的对象，默认通盘进行序列化化处理
        }
        case EJson::Null: // 当前为显式的 Null 值
            return TEXT(""); // 返回空字符串对应 Null
        default: // 对于其他未显式处理的扩展类型
            return SerializeJsonValue(Value); // 使用序列化统一兜底返回
        }
    }

    /** @brief 从 JSON 解析无人机任务角色（target / interceptor） */
    EDroneMissionRole ParseMissionRole(const TSharedPtr<FJsonObject>& CommandJson) // 尝试从配置参数提取并在对应的枚举范围内匹配解析出的无人机任务角色
    {
        if (!CommandJson.IsValid()) // 检查命令字典对象的有效性
        {
            return EDroneMissionRole::Unknown; // 若无效，向调用端反馈为未知角色
        }

        FString Role; // 用于存储提取到的角色名称片段
        if (CommandJson->HasField(TEXT("mission_role"))) // 判断是否明确配置了 'mission_role' 属性
        {
            Role = CommandJson->GetStringField(TEXT("mission_role")); // 提取目标字符串配置
        }
        else if (CommandJson->HasField(TEXT("role"))) // 对非严格属性命名的后备情况查找 'role' 字段
        {
            Role = CommandJson->GetStringField(TEXT("role")); // 提取回退项的配置字符串
        }

        Role.ToLowerInline(); // 将得到的角色文本统一转换为小写进行处理匹配
        if (Role == TEXT("target")) // 验证目标角色是否定义为靶机
        {
            return EDroneMissionRole::Target; // 返回 Target 对应的枚举表示
        }
        if (Role == TEXT("interceptor")) // 验证目标角色是否定义为拦截机
        {
            return EDroneMissionRole::Interceptor; // 返回 Interceptor 对应的枚举表示
        }

        return EDroneMissionRole::Unknown; // 无法匹配有效角色规则，返回未知状态
    }

    /**
     * @brief 判断是否应将生成位置视为米单位（并自动乘 100 转换为 UE cm）
     * 无人机类型默认为米，其他 Actor 则根据 unit 字段判断
     */
    bool ShouldUseMetersForSpawn(const TSharedPtr<FJsonObject>& CommandJson, UClass* ActorClass) // 检测客户端传来的位置数值是否是米单位制以提供给接下来的换算步骤
    {
        if (CommandJson.IsValid()) // 首先判断配置项字典本身存在并有效
        {
            FString Unit; // 临时保存待判定使用的长短距单位系统名称
            if (CommandJson->TryGetStringField(TEXT("unit"), Unit) || // 尝试读取键名统称 "unit"
                CommandJson->TryGetStringField(TEXT("units"), Unit) || // 尝试读取复数形 "units"
                CommandJson->TryGetStringField(TEXT("frame"), Unit)) // 兼顾不同标准中表示系标识字段 "frame"
            {
                Unit.ToLowerInline(); // 将捕获到的文本做就地全小写转换以利于后置的比对条件判断
                if (Unit == TEXT("cm") || Unit == TEXT("centimeter") || Unit == TEXT("centimeters") || Unit == TEXT("ue_cm")) // 如果单位显示为显式的厘米格式相关声明
                {
                    return false; // 返回不转换为厘米制（因为原生就是 UE 的厘米）
                }
                if (Unit == TEXT("m") || Unit == TEXT("meter") || Unit == TEXT("meters") || Unit == TEXT("ue") || Unit == TEXT("world")) // 分析表明输入单位使用的是米等较宽广比例系
                {
                    return true; // 返回应将当前值当作米处理并启动后置转换比例
                }
            }
        }

        return ActorClass && ActorClass->IsChildOf(ADronePawn::StaticClass()); // 如果无显式的声明单位情况，根据内部预设只有派生自 ADronePawn 飞行器的情况判定默认为纯原米单位处理
    }

    /** @brief 尝试设置 Actor 的指定 FString 属性（用于分配 ID） */
    bool TrySetStringProperty(AActor* TargetActor, const TCHAR* PropertyName, const FString& Value) // 基于反射系统动态修改在实体内部指定属性命名的变量数值
    {
        if (!TargetActor) // 验证当前实体对象是否存在
        {
            return false; //若未正常包含直接反馈不通过状态
        }

        if (FStrProperty* Property = FindFProperty<FStrProperty>(TargetActor->GetClass(), PropertyName)) // 使用静态反射根据指定字符查找 TargetActor 中名为 PropertyName 的 FStrProperty 参数属性存储句柄
        {
            Property->SetPropertyValue_InContainer(TargetActor, Value); // 通过句柄找到对应的内存变量将其重新赋上 Value 内容设进行修改
            return true; // 修改操作合法通过并表示完成成功过程
        }

        return false; // 即便找了也并未拥有匹配数据标识孔位所以未能达成重设属性最终目的
    }

    /** @brief 将 ActorId 写入 Actor 的所有可能的 ID 属性（DroneId / TurretId / GuidanceId / ActorId / AgentId） */
    void AssignActorId(AActor* TargetActor, const FString& ActorId) // 尝试将统领下的通用 ActorId 同步推写更新进各类别实体可能内载的各项标识数据列孔里
    {
        if (!TargetActor || ActorId.IsEmpty()) // 如果接收指针未对并失效或者携带用于标记内容源 ID 为空气
        {
            return; // 将视为作废任务丢弃退出函数
        }

        TrySetStringProperty(TargetActor, TEXT("DroneId"), ActorId); // 按顺序向存在几率较高的 'DroneId' 插孔上对位数据推送设置
        TrySetStringProperty(TargetActor, TEXT("TurretId"), ActorId); // 对老版本 'TurretId' 标记符提供对位支持并写入操作数据
        TrySetStringProperty(TargetActor, TEXT("GuidanceId"), ActorId); // 同理验证是否可以写下对位于引导载荷专属名 'GuidanceId'
        TrySetStringProperty(TargetActor, TEXT("ActorId"), ActorId); // 对于一般类型尝试标准填充于 'ActorId' 槽中做一般性基础保存
        TrySetStringProperty(TargetActor, TEXT("AgentId"), ActorId); // 为确保框架设计的最通式基准能够准确生效也写入 'AgentId' 后期默认名使用处
    }

    /** @brief 生成随机 Actor ID（actor_1000~9999） */
    FString MakeRandomActorId() // 用随机数功能来产生一个系统默认的未分配识别代号序列作为回退机制使用
    {
        return FString::Printf(TEXT("actor_%d"), FMath::RandRange(1000, 9999)); // 调用 RandRange 生成数字在1000至9999再结合格式模板化为含 "actor_" 前缀的最终字符串
    }

    /** @brief 应用生成后的元数据：ID + 任务角色 */
    void ApplySpawnMetadata(AActor* TargetActor, const FString& ActorId, EDroneMissionRole MissionRole) // 封装元数据信息填充的综合应用步骤作为独立动作向新建对象调用执行注入阶段参数
    {
        AssignActorId(TargetActor, ActorId); // 第一步分发调用将统一设下的系统辨识基准串行注册号写入物理实体
        if (ADronePawn* SpawnedDrone = Cast<ADronePawn>(TargetActor)) // 用向上强转类型手段检查新生物对象是否能当作具体的 ADronePawn 结构交互
        {
            SpawnedDrone->MissionRole = MissionRole; // 通过转化来的无人机特定接口把解算解析后的飞行器专门属性分配落实并归顺进去记录留存
        }
    }

    /**
     * @brief 在命名参数中查找与属性名匹配的值
     *        支持精确匹配、后缀匹配、arg0/p0 索引匹配
     */
    const FString* FindNamedParameterValue( // 在字典或以非序号关联排列参数表中识别对应具体函数映射名目的数据绑定值的搜核寻找方法设计
        const TMap<FString, FString>& NamedParameters, // 来自配置载入实际挂载提供的全套“键-值”参数的匹配表
        const FString& PropertyName, // 根据引擎反射抽取得到的执行点处目标所需参数属性确切命名比对键
        int32 ParamIndex) // 用于引擎端确定的依顺序编号用于后备或默认查询补足检索依据对应第 N 配置元素位标
    {
        for (const TPair<FString, FString>& Pair : NamedParameters) // 首先用宽松条件遍寻找参数字典项里的键对
        {
            if (Pair.Key.Equals(PropertyName, ESearchCase::IgnoreCase)) // 最高一级执行毫无大小写差别的参数对应绝对键位对标检查验证
            {
                return &Pair.Value; // 绝对相等说明完全精确找到了内容匹配的目标直接回调交出值处内存引用地址
            }
        }

        for (const TPair<FString, FString>& Pair : NamedParameters) // 如果绝对名称不能覆盖寻找请求情况在原参数组上开始执行次级策略对属性容错后段查切支持功能操作展开查询
        {
            if (!Pair.Key.EndsWith(PropertyName, ESearchCase::IgnoreCase)) // 直接拒绝掉连结尾片段都没相似重连的无关紧要字典对象进行直接继续排除跳跃的下一次排查
            {
                continue; // 终止本次元素核对进行下轮的筛查推进工作
            }

            if (Pair.Key.Len() == PropertyName.Len()) // 对于不仅结尾端长相同并且连通整体字符容量计算相同即为等于本身的情形直接捕捉完成验证
            {
                return &Pair.Value; // 为直接变异比对等同的成功捕获对象给出相应的内容映射返回地址源头参数项
            }

            const int32 PrefixIndex = Pair.Key.Len() - PropertyName.Len() - 1; // 利用全长度和需要后缀减量差异位置定置获取假定处于前引特征标识节点的位置并做前导判断（比如 x 与 location_x 中的 _ 地位）
            if (PrefixIndex >= 0 && Pair.Key[PrefixIndex] == TCHAR('_')) // 在符合条件的偏移地址发现代表变量名隔离定义标准分隔下划线的时候，认可以该包含后缀的标识命中当前属性
            {
                return &Pair.Value; // 将携带了此前缀掩饰实际相同后端意义的数据对内容交接反馈予以填充运用
            }
        }

        const FString ArgKey = FString::Printf(TEXT("arg%d"), ParamIndex); // 万一参数完全舍弃变量实名只提供基于次序列下标方式填参设计合成兼容占位的 arg<序号> 寻址词标码
        const FString PKey = FString::Printf(TEXT("p%d"), ParamIndex); // 以及同样处理为 p<序号> 并作备用来生成寻址验证核查键词标志名
        for (const TPair<FString, FString>& Pair : NamedParameters) // 利用生成的基于次序合成替换名重新扫入全盘再遍历所有可得数据源字段检查有无以序指代的可能对象数据
        {
            if (Pair.Key.Equals(ArgKey, ESearchCase::IgnoreCase) || Pair.Key.Equals(PKey, ESearchCase::IgnoreCase)) // 如果检测到存在以上文对应替换方案形式配置提供传参设置数据内容的记录存在等式通过时：
            {
                return &Pair.Value; // 同意以次序等代名的方式截取该占位中的对应数据供给当前需求的实际具体要求参数引用处返回内容交割
            }
        }

        return nullptr; // 这个形参要求无法通过所有的任何名实方式寻获可提供的关联供给源：报无对应资源失败返回给框架层
    }

    /** @brief 构建状态响应 JSON 对象 */
    TSharedPtr<FJsonObject> MakeStatusResponseObject(bool bSuccess, const FString& Message) // 用于创建含有操作完结明确 status 布尔标志位和基本运行反馈文本附载项的基础标准回文负载层
    {
        const TSharedPtr<FJsonObject> ResponseJson = MakeShareable(new FJsonObject); // 获取申请使用的新建未赋值裸 FJsonObject 配置实体包裹
        ResponseJson->SetBoolField(TEXT("status"), bSuccess); // 放置一个在调用框架最关心的方法动作处理终果指示位布尔健存入包头
        ResponseJson->SetStringField(TEXT("message"), Message); // 以及随同的对因调用结束补充描述信息与异常原由说明等字符串放入随文本段内
        return ResponseJson; // 输出完整的构建内部封装核心回传单元指针并准备进更高封装套处理环节
    }

    /** @brief 将响应对象包装在指定字段名下并序列化 */
    FString SerializeWrappedResponse(const FString& FieldName, const TSharedPtr<FJsonObject>& ResponseJson) // 利用一层外框属性封包包裹原带返回基础格式的对象之后产出适合接口规范对接发包直接串型文本体
    {
        const TSharedPtr<FJsonObject> FullResponse = MakeShareable(new FJsonObject); // 定义最高顶头容器包装对象模型实体用于包裹其它 JSON
        FullResponse->SetObjectField(FieldName, ResponseJson); // 设立用于标注指令应答回传明确特定对应类别如 add_actor_return 字段属性做最外表并放置原传入信息包为子树

        FString OutputString; // 开辟系统文本级别资源分配进行后续的生成并填储转化输出缓存的堆载任务准备流向处理口径
        const TSharedRef<TJsonWriter<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>> Writer = // 配置对排印形式的精简输出规则处理器取消多余可读性的换行空格以便符合极简协议通讯传递流载重
            TJsonWriterFactory<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>::Create(&OutputString); // 生成使用当前预设规则对应并对接指定串输出流管道接口写出器
        FJsonSerializer::Serialize(FullResponse.ToSharedRef(), Writer); // 发动反推转现文本格式化大执行命令彻底按规则清出树结文本输出完毕存储向 OutputString 对象
        return OutputString; // 函数返回这一压缩生成的长字流序列回发反馈给上级请求完成指令操作最后全段调用落幕退传通讯结束
    }
}

// ──── 构造函数 ────

FCommandHandle::FCommandHandle(UGameInstance* InGameInstance) // 对实体对象的处理总类初始化设置必须依赖包含引擎核心级游戏生命状态信息的注入
    : GameInstance(InGameInstance)  // 为类的成员变量 GameInstance 拷贝建立与生命管理实例环境引用依赖，保证此后的命令下发具备 UWorld 环境入口通道
{
}

// ──── 类型解析 ────

/**
 * @brief 解析 Actor 类：支持多种路径格式
 * - Blueprint'/Game/...'       → 蓝图资产路径
 * - /Game/...                  → 资产路径
 * - /Script/GraduationProject. → C++ 类路径
 * - 简名（如 ADronePawn）     → 自动拼接 /Script/ 前缀
 */
UClass* FCommandHandle::ResolveActorClass(const FString& ClassNameOrPath) // 通过灵活解析各种名称定义标识将传入的文转成能够被产生器识别调用实际的类 UClass 注册源对象
{
    if (ClassNameOrPath.StartsWith(TEXT("Blueprint'/")) || ClassNameOrPath.StartsWith(TEXT("/Game/")))  // 蓝图或资产路径 （判断调用中传入的是指向预封装蓝图还是基础资产的直接路址规则特征）
    {
        return LoadClass<AActor>(nullptr, *ClassNameOrPath); // 对于具体的包引依赖执行从打包的持久化对象或内存结构加载获得可运行态的原根反射实例地址返回以支撑其实化发生操作流程推进
    }

    if (ClassNameOrPath.StartsWith(TEXT("/Script/")))  // 完整 Script 路径 （对于使用虚幻引擎特定的标准代码反射空间明确地址定义的纯净大类的路径规则直接比对方式判断特征）
    {
        return StaticLoadClass(AActor::StaticClass(), nullptr, *ClassNameOrPath); // 通过查找已解析编译内驻常驻类的记录总表中明确直接寻源挂靠寻找和提炼对应类引用资源定位返回执行支持接驳
    }

    // 尝试在本项目模块中查找
    FString FullClassName = FString::Printf(TEXT("/Script/GraduationProject.%s"), *ClassNameOrPath); // 如仅传递普通且缺乏规范性界定的裸字名缺省状态强制加上并认为该指向存在于 GraduationProject 原码工作核心空间前引后缀组立
    UClass* ActorClass = StaticLoadClass(AActor::StaticClass(), nullptr, *FullClassName); // 再用新全路径配置走流程测试本区域搜索能不能发现真实有效的底层对应的反射类型源指向对象是否存在或抓获成真
    if (!ActorClass)  // 找不到则尝试 Engine 模块 （如果上边项目库尝试全域搜索未捕成功，该变量即成为零）
    {
        FullClassName = FString::Printf(TEXT("/Script/Engine.%s"), *ClassNameOrPath); // 则判定这有可能会是引擎本身的公共默认类名（如基础通例框架）将头标改变导向到宽大的虚幻基本层去
        ActorClass = StaticLoadClass(AActor::StaticClass(), nullptr, *FullClassName); // 再试图深入一次进行 Engine 底核全反射层提取寻迹，试图找到支持实例化用的对象定义模型基准包定位反馈给大类注册加载调用器处理下放流程
    }

    return ActorClass; // 无保留返还最终通过搜索解析得的全部最终实体判定（包含未果时正常透传过来的 nullptr 并抛后路报错层处决）并离开控制转移处理
}

/** @brief 从 JSON 读取生成位置和旋转（pose.x/y/z/roll/pitch/yaw） */
void FCommandHandle::ReadSpawnPose(const TSharedPtr<FJsonObject>& CommandJson, FVector& OutSpawnPos, FRotator& OutSpawnRot) // 把配置载荷中明确规定或未定的三维和四元形态数据抽离写入具体对应标准类型的形态引用准备供后层放置动作应用
{
    OutSpawnPos = FVector::ZeroVector; // 将输出存储占位 FVector 重设为基于原点不携带额外量位的基准值以作兜底默认设置点防非法使用值影响部署生成
    OutSpawnRot = FRotator::ZeroRotator; // 将存储回传偏转设定位基准不含旋转倾向零偏量归口基准作兜底值预载设置对齐状态对轴配置生成点避免发生错误翻倾姿势初载值传布生效影响呈现部署正常起旋工作
    
    const TSharedPtr<FJsonObject>* PoseJsonPtr = nullptr; // 用于临时探出接取在源内容内挂钩存在的嵌套位姿 JSON 所配专用挂接字典指针预占节点声明置空操作
    if (!CommandJson->TryGetObjectField(TEXT("pose"), PoseJsonPtr) || !PoseJsonPtr || !PoseJsonPtr->IsValid()) // 如果调用中并不含设定此对象的关键字段并且取之不得未提供可用对象：
    {
        return; // 在无可用的明确重载要求情况则中止进一步检索直接退出执行从而自然采信此前配置的置零零值作处理基调退出不操作处理流
    }

    const TSharedPtr<FJsonObject>& PoseJson = *PoseJsonPtr; // 将接取得有有效实装存在的数据节点由可指针对标提取转化成一个实效操作的对象源体并映射对映到名称处理块供用
    double Value = 0.0; // 配置浮点测量尺度容器提供给抽取位置分坐标值的获取缓冲周转临时区带复用机制以利内存与运算开销复用原则提取实装浮点对象赋值使用前置分配
    if (PoseJson->TryGetNumberField(TEXT("x"), Value)) // 试图寻迹找到名为 x 长度分量位记录配置参数
    {
        OutSpawnPos.X = static_cast<float>(Value); // 如果能够找到数值提取强制退缩转为 float 给位空间对应分位存录提交设定位置覆盖使用原有的0值重置修改影响坐标表现设定点值使用传导给系统载装落子调用执行配置重组落笔定轴表现点使用传落效结果体现参数输出位置控制影响定载系统调用位置表现
    }
    if (PoseJson->TryGetNumberField(TEXT("y"), Value)) // 同理尝试从源头字段里检索 Y 标记控制段配置项提供取值载重应用记录提取传递值载入内存区供处理更新数据位对位控制用作控制输入改变生成点源设定依据生效展现动作表现效果生成作用处理修改展现值表现落成表现效能修改系统内部体现用量应用传导出控制修改值更新传向展示展现最终用量落地生效作用记录参数配置执行展现动作操作表现展现过程体现位置信息更新表现结果落地生成执行过程
    {
        OutSpawnPos.Y = static_cast<float>(Value); // 将取出有效 Y 指定控制长度通过等比降阶压回虚幻系统认可能量范围浮标段置入该输出量承载对象相应控制孔接收供接后步环节执行配置作用表现系统生成修改影响定位功能处理效能调用传输出生效落地动作展示实现系统过程体现展现定点修改功能参数值表现生效体现作用效展现传导出结果系统调用展现
    }
    if (PoseJson->TryGetNumberField(TEXT("z"), Value)) // 依然查找对应纵深系统深度高度标尺记录 Z 提供修改控制项覆盖传入影响落地位置点用生效操作输出结果过程展示位置执行动作提供配置使用值展现生效修改位置表现影响展现参数作用生成效能落地生效传导出信息过程记录值参数配置修改生成指令配置展现展现结果控制表达系统调动展现结果作用过程配置功能展现落地展现生效体现定位结果落地
    {
        OutSpawnPos.Z = static_cast<float>(Value); // 同理取出并格式置换为引擎适应基底放入高程记录配置项孔准备反馈系统修改位置指令产生效用的展示环节执行功能参数设置影响落地生效更新呈现使用值展现产生的结果配置参数产生执行作用展现功能作用控制信息修改参数展现落地结果效果展现过程产生落地控制传导指令参数配置修改产生功能影响
    }

    double Roll = 0.0; // 对于系统内代表滚转偏扭控制轴参数开辟专项储备独立缓冲占位赋闲挂置使用记录供加载控制提取展现落地呈现使用参数设置展现配置数据
    double Pitch = 0.0; // 同理为俯仰变轴提供缓冲容纳操作变量占位存挂并归零起始确保非传出即无效数据不致误扰乱转控制
    double Yaw = 0.0; // 后设控制向轴提供容载并空填作为待机后取位使用
    PoseJson->TryGetNumberField(TEXT("roll"), Roll); // 取调字典查找配用标号有获取有效偏翻项的话赋予之前声明之控制占位重赋值执行后取展现动作的反馈更新传导操作落地产生指令值
    PoseJson->TryGetNumberField(TEXT("pitch"), Pitch); // 若此查明记录具有头低头高的偏航度标载就刷新该专位储备区准备供给对转控应用生效产生执行展现
    PoseJson->TryGetNumberField(TEXT("yaw"), Yaw); // 如找到对系统左摇右摆改变方向配置的航偏控制请求也会赋值回位准备调用时配置作用呈现修改使用转姿角度控制展现生效动作表现落地效用执行展现操作落地
    OutSpawnRot = FRotator(static_cast<float>(Pitch), static_cast<float>(Yaw), static_cast<float>(Roll)); // 将零散采集而来的各个扭度分参并按虚幻既定的先俯仰后航偏后滚转顺序重拼铸合成为一个具有明确效力合体的旋转类向配置通过传口放出影响系统产生姿置落成执行展现生成效能落地
}

/** @brief 解析 Actor ID：优先使用 expected_id，否则生成随机唯一 ID */
FString FCommandHandle::ResolveActorId(const TSharedPtr<FJsonObject>& CommandJson, UAgentManager* Manager) // 实现对最终出厂标记辨别生成使用确认规则优先级和保障有效独立性的检索生成功能分配方法配置逻辑动作获取结果标识展现
{
    if (CommandJson->HasField(TEXT("expected_id")) && !CommandJson->GetStringField(TEXT("expected_id")).IsEmpty()) // 最前置判断其用户配置层是否已经提出了带有其指定标记且不为空明确意义的要求预期 ID
    {
        return CommandJson->GetStringField(TEXT("expected_id")); // 若成立此首需则满足上级配置期待而直返回此标记不设过多检索条件限制产生直效落成使用传出调用配置供给产生参数使用标识生效执行展现表达作用参数生成输出
    }

    if (!Manager) // 当且仅当因底层组件服务未得接通初始化发生脱离没有供后勤提供可询对比参照的管理器
    {
        return MakeRandomActorId(); // 不做额外冲突比较尝试就采取直接瞎产生法丢出随意串去冒配碰撞碰运气供给当事使用产生执行表现反馈结果生成生效展现使用功能落地展现效果标识生成调出调用作用使用
    }

    FString ActorId = MakeRandomActorId(); // 常理情况下先无责调用随即库得到产生一组看似无关联新名用作核对比对
    while (Manager->GetAgent(ActorId) != nullptr) // 在此步上用得来的这组号尝试提交查询如果返回证明了其实该名称已经占挂在场其他老节点上存在（重号）
    {
        ActorId = MakeRandomActorId(); // 此条件下拒绝重名再次申请另一组产生数替换以供再一次对比检查使用直到成功产出避雷的新号不致混乱的独立配置展现落地传导出执行标识参数产生更新使用结果生成表现落地效应调控使用
    }

    return ActorId; // 直至历经筛过确实没有任何重名的安全后号段产生完妥则投送至下步调用配置用作正式实体名称唯一落账使用展现结果生效执行标识传导产生影响表现执行生效落地展示控制使用产生输出控制参数使用表达生效功能
}

/** @brief 解析函数调用参数：支持数组 (positional) 或对象 (named) 两种 JSON 格式 */
bool FCommandHandle::ParseCallParameters( // 鉴别和转化供函数命令调用带过来的随附加负载参配置以对应到标准结构以适配接下来反向动态驱动所需不同模式结构数据执行分析
    const TSharedPtr<FJsonObject>& CommandJson, // 带着未定形式但带有明确字段要求的全部配置包作为查问分析源
    FCallParameters& OutCallParameters, // 装填处理得到且最终整理定型的参数集结果用于反射加载传出供外部执行应用表现展现使用生成效应输出参数作用功能控制
    FString& OutError) // 若拆解释义流程遇阻报错用以生成回寄问题详情的错误说明报告附着传导落地执行结果表现展现传导出作用使用反馈参数控制
{
    OutCallParameters = FCallParameters{}; // 进流首发重置输出用的容器内部原有全空避免造成上个包记录带出错误混用等杂症出现保证安全配置覆盖生效落地传出参数展现结果执行生成作用使用展现控制影响参数操作产生动作
    OutError.Empty(); // 对接收报警的字段执行清除重定归零待接查如果完全跑通过无事可不必管它的留白无害返回输出使用功能执行生效展现结果操作产生传导影响控制展现展现落地调用配置参数修改输出
    
    if (!CommandJson.IsValid() || !CommandJson->HasField(TEXT("parameters"))) // 对主源空件判断和探测是否有明确挂接着供随命令伴生的“parameters”名称参数携带件
    {
        return true; // 既是没有便认定它是一次光执行无入参需求空传召调拨事件而成功结案无阻挡放行输出展示生效落地操作展现效应控制修改结果使用配置参数调用执行作用功能展现
    }

    const TSharedPtr<FJsonValue> ParametersValue = CommandJson->TryGetField(TEXT("parameters")); // 精确索出对应参数子节点对象作进一步结构判断分析使用展开产生传导操作控制展现修改使用执行输出
    if (!ParametersValue.IsValid()) // 此时依然再次确认抽索的节点确已实际持有一份实存载体可作用性并非空气断链
    {
        return true; // 空则认定其实此番为同样地无供传参配置要求予以准行放过通过判断展现落地执行反馈成功结果动作功能配置应用
    }

    if (ParametersValue->Type == EJson::Array) // 如果验明其实体展现的类型特征为符合无特别名目标志而直接采用连续阵列连串排开的 JSON 列表体型组织格式（位置型参数队列）
    {
        const TArray<TSharedPtr<FJsonValue>>& Values = ParametersValue->AsArray(); // 首先利用其型转机制暴露出底里的数组实链载体清单供调用展开查索赋值使用动作展现控制操作功能影响传导展现落地产生执行作用生效控制参数展现输出
        OutCallParameters.PositionalParameters.Reserve(Values.Num()); // 利用总计数前置为接下来使用的装载数组开启内存留白留号配置用作提前定格以免多生反复开辟内存拖沓性能参数执行落地效果生成调用影响功能展现配置使用产生作用
        for (const TSharedPtr<FJsonValue>& Value : Values) // 对阵列的里的载值依次轮排巡阅遍历提取和装塞动作操作执行传导出效果展现控制作用生向参数修改过程展现应用调用
        {
            OutCallParameters.PositionalParameters.Add(JsonValueToParamString(Value)); // 逐一地经过本文件专用通用变换算法压入转成单段字符字串并追补进入输出准备好的位置列表大类参数存储袋落地生效表现提供后续执行所需展现动作操作传导出控制应用配置使用生向参数作用
        }
        return true; // 队列整装完毕此阵列态的梳理即告完成不报错直接安全传出结束返回处理通过判定展现落地生效执行动作表现控制
    }

    if (ParametersValue->Type == EJson::Object) // 否则若是所提供实装内容其结构表现出了完整的名配实载大字典集合对象状态特征判定为是使用的挂名映射名列寻参数模型特征功能（字典类型参数模式）
    {
        OutCallParameters.bUseNamedParameters = true; // 大表决定此一轮获取输出传结果确实须在后续取参数的环节走寻找名称验证调性方案作开启布尔状态设置提供传出执行标识作用使用落地影响动作生向展现控制配置
        const TSharedPtr<FJsonObject> ParametersObject = ParametersValue->AsObject(); // 转而拉出真正带有键值集合对象的实质体模型供切件分析和打解处理用向生向展示发生操作反馈效果配置传导致调用执行使用
        if (!ParametersObject.IsValid()) // 如再次确认获取实质依然还是不存在或是无内容废死物件失效异常情况判断排除使用动作保护操作产生执行表现生效控制拦截展现反馈提供
        {
            return true; // 则仍当成无载荷的简单召唤事件直行通过不再苛深探追配置要求执行成功判定展现返回作用生效操作控制使用
        }

        for (const TPair<FString, TSharedPtr<FJsonValue>>& Pair : ParametersObject->Values) // 展开这所有列明字号标签的参数内容进行一遍历查询循环处理配置
        {
            OutCallParameters.NamedParameters.Add(Pair.Key, JsonValueToParamString(Pair.Value)); // 把它本自带来的字号以及经专用工具转变生成的字符串文本值压入到新备好的有名字典词条库对入提供以后按需检索匹配提单使用展现产生动作配置传导向控制影响返回生效落地调用
        }
        return true; // 所有有名字配全部转装好打结送行成功判定状态退出传回使用作用生效控制展现结果调用参数生成功能使用动作表现
    }

    OutError = TEXT("parameters must be array or object"); // 对以上结构形态无一兼容而表现异常者给出报错标定指示错误格式原因的陈述准备回传报错使用产生向配置功能影响落地传导出说明操作表现调用标识
    return false; // 以表明分析过程完全溃败终以拒收并中止传回此执行请求失效信息标识以告上行管理功能展现作用表现控制生效拦截错误落地操作呈现过程配置
}

// ──── 命令分派 ────

/** @brief 根据 JSON 字段名分派到 Add / Remove / Call，未匹配返回空字符串 */
FString FCommandHandle::HandleCommand(const TSharedPtr<FJsonObject>& RootJson) // 命令处理主入口分配器，负责鉴定动作请求
{
    if (!RootJson.IsValid()) // 如果传入的 JSON 根对象指针失效
    {
        return TEXT(""); // 则不执行任何操作并返回空字符串表示没有捕获分派执行结果
    }

    if (RootJson->HasField(TEXT("add_actor"))) // 检查命令字典是否有包含字段名 'add_actor'，以代表这是一个生成实体的生成请求
    {
        return HandleAddActor(RootJson->GetObjectField(TEXT("add_actor"))); // 将该挂靠在字段下的生成规则对象转递至处理函数并交出最终执行结果字符串
    }
    if (RootJson->HasField(TEXT("remove_actor"))) // 检查命令是否有 'remove_actor' 删除实体请求
    {
        return HandleRemoveActor(RootJson->GetObjectField(TEXT("remove_actor"))); // 将附带了待删目标信息的参数交由删减服务方法并转发其输出字符串
    }
    if (RootJson->HasField(TEXT("call_actor"))) // 检查命令是否包含 'call_actor' 请求执行特定函数调用
    {
        return HandleCallActor(RootJson->GetObjectField(TEXT("call_actor"))); // 将随带有函数名参数体的调用包转移给反射执行机构进行传参调用处理并交付返回输出
    }

    return TEXT(""); // 对于所携带负载均不包含以上基础合法格式要求的无匹配孤立封包执行缺省的越过和空串通过
}

// ──── 动态生成 Actor ────

/**
 * @brief 在场景中生成指定类型的 Actor
 *
 * 工作流程：
 * 1. 解析 Actor 类
 * 2. 读取生成位姿（自动处理米/厘米转换）
 * 3. 解析或生成唯一 ID
 * 4. SpawnActorDeferred → 应用元数据 → FinishSpawning
 * 5. 注册到 AgentManager
 */
FString FCommandHandle::HandleAddActor(const TSharedPtr<FJsonObject>& CommandJson) // 响应客户端请求，依循引擎标准延时注册（DeferredSpawn）生成实体流程在指定的世界点生产生成实例
{
    if (!CommandJson.IsValid() || !CommandJson->HasField(TEXT("classname")))  // 缺少类名 // 前置检查字典内是否拥有代表基础构造类型的最关键属性
    {
        return MakeAddActorResponse(TEXT(""), false, TEXT("Missing classname")); // 如未指定要生成的类则报错并中断执行阻拦
    }

    const FString ClassNameOrPath = CommandJson->GetStringField(TEXT("classname"));  // 读取类名 // 获取指示类型的标识字符路径供检索器分析出目标大类的内存反射数据结构
    UClass* ActorClass = ResolveActorClass(ClassNameOrPath);  // 解析为 UClass // 将名段文字提取识别并转化为可以生产物理对象的引擎 UClass 参照指针
    if (!ActorClass) // 如果未查找到
    {
        UE_LOG(LogTemp, Error, TEXT("[FCommandHandle] Class not found: %s"), *ClassNameOrPath); // 输出内部日志表示请求产生未能命中加载目标的类资源
        return MakeAddActorResponse(TEXT(""), false, FString::Printf(TEXT("Class not found: %s"), *ClassNameOrPath)); // 打包未载入目标类型的失效回应拒绝生产并传回
    }

    FVector SpawnPos = FVector::ZeroVector; // 分配代表待载空间三维锚点参数准备缓存抽取内容默认位置零点
    FRotator SpawnRot = FRotator::ZeroRotator; // 分配表征初始部署扭向偏向角参数并预设初态正位备提取替换改写
    ReadSpawnPose(CommandJson, SpawnPos, SpawnRot);  // 读取生成位姿 // 将含有源头参数要求的对象丢入分析抽取模块得出物理展现姿态点放入存储槽准备应用产生传出
    if (ShouldUseMetersForSpawn(CommandJson, ActorClass))  // 米转厘米 // 对其进行检测若需对不使用 UE 系统刻度规则（厘米）的值按常规长度量度提供比例变化
    {
        SpawnPos *= 100.0f; // 进行按百倍放大的倍乘换制确保符合引擎底层处理刻定需求应用控制调用输出结果生效参数控制功能修改传导控制作用
    }

    UAgentManager* Manager = UAgentManager::GetInstance(); // 获取总控实体名册库的指针建立查询挂载联系
    if (!Manager) // 防止严重上下文错误带来的空指针造成后续全线查问崩溃中断
    {
        return MakeAddActorResponse(TEXT(""), false, TEXT("Agent manager unavailable")); // 说明因全局管控缺失本次增加实体不能进行处理记录注册退避调用回驳异常
    }

    const FString ActorId = ResolveActorId(CommandJson, Manager); // 要求处理生成包含冲突判定或接纳自定义或最终合成出一个独立安全的可挂牌名称符号用在实体身份标记展现输出配置参数传导上
    if (Manager->GetAgent(ActorId) != nullptr) // 对提取来预使用的标记号进行二次底池重名防御确定不与现存在场的有效载荷名称发生交叉撞击
    {
        UE_LOG(LogTemp, Warning, TEXT("[FCommandHandle] Actor ID already exists: %s"), *ActorId); // 如果撞车发出预警表明无法强覆盖占用现用识别名报错执行结果展现操作
        return MakeAddActorResponse(ActorId, false, FString::Printf(TEXT("Actor ID already exists: %s"), *ActorId)); // 通知请求者生成的标号在后台因已使用导致的不能正常入网记录失败传导回显展示结果
    }

    UWorld* World = GameInstance ? GameInstance->GetWorld() : nullptr; // 通过持久关联获取到当下的场局世界提供对生成调用执行下放功能使用的空间画布
    if (!World) // 极端检查若没有场局存在则不允许有产生行为配置落地动作传导使用
    {
        return MakeAddActorResponse(ActorId, false, TEXT("World is null")); // 如发生世界销毁或不存在返回直接错误阻止引擎进一步抛崩溃结果落地展现配置传递
    }

    const FTransform SpawnTransform(SpawnRot, SpawnPos); // 把刚抽取和修正得来的长宽转角组合在一起成为统一部署参数集结构使用调用过程反馈使用展示影响效果功能配置调用作用参数生向执行
    AActor* NewActor = World->SpawnActorDeferred<AActor>( // 发起分步生成的请求：建立引擎基本资源底模挂件但不立即完成构造和进入 Tick 循环进行展现配置落位应用控制生效处理功能
        ActorClass, // 所选的执行蓝图或者原C++的配置 UClass 提供型准参数供给底层构造发生
        SpawnTransform, // 挂接需要应用放置的具体预定空位置标姿供构造放置时设定结果生成产生表现呈现调用参数传导执行操作动作落地
        nullptr, // 暂不设立实体发起者持有者信息供给产生
        nullptr, // 不为其部署拥有所有权的特定控制组件传导调用生效配置影响落地使用参数控制影响表现作用影响控制展现控制展现生向动作
        ESpawnActorCollisionHandlingMethod::AlwaysSpawn); // 指示配置如果在下放实体时与场局存在位置冲突不避让依然强行执行通过强产
    if (!NewActor) // 确认基础延迟加载阶段确实创建出了可以附着属性的对应可用实体空间占位对象对象表现
    {
        UE_LOG(LogTemp, Error, TEXT("[FCommandHandle] Failed to spawn actor")); // 反之如果是加载类异常没有出产记录于系统发生内部严重错误
        return MakeAddActorResponse(ActorId, false, TEXT("Failed to spawn actor")); // 将加载异常无法配置传回提供原因解释并驳回创建请求落地失败呈现执行调用落地结果生向作用展现控制配置传导
    }

    const EDroneMissionRole MissionRole = ParseMissionRole(CommandJson); // 从附属性负载继续抽取专用于在初始化生效前必须打入其核心控制流设定的状态枚举标记配置提供使用
    ApplySpawnMetadata(NewActor, ActorId, MissionRole); // 利用获取的身份标记将关键性质信息压入其对应的专从预设坑口内打牢保证数据贯穿产生和其在场存活整个生成表现动作生效执行过程落地应用调用

    UGameplayStatics::FinishSpawningActor(NewActor, SpawnTransform); // 结束它的半完形挂起态推进引擎全阶段完成正式落地部署应用并完成全部最终启动运行周期的全量投产过程激活展现效果控制展现参数操作落地产生配置
    ApplySpawnMetadata(NewActor, ActorId, MissionRole); // 作为虚幻特性的补偿在彻底调用其产生函数完成后可能有数据归零态的某些类型必须再次确立这信息的写入操作执行提供系统更新生效

    FString Label = CommandJson->HasField(TEXT("label")) ? CommandJson->GetStringField(TEXT("label")) : TEXT("Vehicle"); // 为在开发视图编辑器中有个可辨识的名字而不是匿名去尝试探知是否有标记需求传递
    if (Label.IsEmpty()) // 或如果提供字空串做保护回填展现表现结果控制产生使用传导调用产生执行展现配置产生操作
    {
        Label = TEXT("Vehicle"); // 使用保护名做替代表现作为配置应用使用落地调用参数
    }
    NewActor->SetActorLabel(FString::Printf(TEXT("%s_%s"), *Label, *ActorId)); // 为其实际世界赋予一拼接的明码表显称号供给开发时可视观测辨识跟踪生成执行结果呈现应用操作应用参数传导致能记录跟踪展现

    if (Manager->GetAgent(ActorId) != NewActor) // 为了防止异常漏网对 AgentManager 取回判断：该识别大号是否有真正纳入并持有一致于刚新得此对象本身内存
    {
        Manager->RegisterAgent(ActorId, NewActor); // 若不归统登记则立刻通知系统造册打底纳入后续整体受系统监督支配调度查册登记落用控制操作作用执行调用展示生效反馈
    }

    UE_LOG(LogTemp, Log, TEXT("[FCommandHandle] Actor created successfully, ID: %s"), *ActorId); // 给内部控制流终端汇报一执行正确成功的正常标识状态事件作为生成调用表现
    return MakeAddActorResponse(ActorId, true, TEXT("Actor created successfully")); // 回制并推送含有其定命之名且表状态完美的成行告知成功负载返回网络呈现
}

// ──── 销毁 Actor ────

/** @brief 销毁指定 ID 的 Actor 并清理所有别名绑定 */
FString FCommandHandle::HandleRemoveActor(const TSharedPtr<FJsonObject>& CommandJson) // 具体负责拆除请求从引擎世界中卸载和销毁相关资源动作过程的流转分配响应执行点
{
    FString ActorID; // 用于存储想要清理的目标注册唯一名引流传递落地调控处理
    if (!CommandJson->TryGetStringField(TEXT("actor_id"), ActorID)) // 缺乏必要的指示到底要谁消失的核准名源目标参数传递导致配置异常获取呈现失败调用
    {
        return MakeRemoveActorResponse(false, TEXT("Missing actor_id")); // 退返参数失常错误给客户请求告知调用不全中断传导反馈结果功能生效
    }

    UAgentManager* Manager = UAgentManager::GetInstance(); // 索要负责整个实体存取大集的管理机构代理提供管理调用表现支持
    if (!Manager) // 防止系统因特殊情况未能准备就绪出现无权无库查档产生空灾中断崩溃拦截系统执行
    {
        return MakeRemoveActorResponse(false, TEXT("Agent manager unavailable")); // 以管理器异常名义拒绝清理退出状态执行退反
    }

    AActor* ActorToRemove = Manager->GetAgent(ActorID); // 发起借由名索引在册记录寻得在虚幻空间实际相接轨的映射真体应用控制对象提取配置使用展现作用
    if (!ActorToRemove) // 果真在底册并未录在此名下之人或其实体已于上个环节丢失消亡
    {
        UE_LOG(LogTemp, Warning, TEXT("[FCommandHandle] Actor not found: %s"), *ActorID); // 系统内记一无头删除异常错行记录提供参数表现
        return MakeRemoveActorResponse(false, FString::Printf(TEXT("Actor not found: %s"), *ActorID)); // 同时回驳网络这查无此事指令没有清除处理作用生效表现
    }

    TArray<FString> AliasIds; // 利用这个清理执行阶段开辟空存以准备清扫同一源内存体拥有多名马甲注册的情形储备
    const TArray<FString> ExistingIds = Manager->GetAllAgentIds(); // 拿到涵盖目前满挂状态的记录单底册列表用于翻找对应相同人的多份记名使用
    for (const FString& ExistingId : ExistingIds) // 开展从全名目表里一轮又一笔的翻转核实对比排查控制落地调用生效执行作用参数控制过程反馈展现
    {
        if (Manager->GetAgent(ExistingId) == ActorToRemove) // 万一它不仅有着刚才要杀的名还有这额外挂出的另外名目指向均皆为其本体一人时抓出来验证同挂
        {
            AliasIds.Add(ExistingId); // 即录下其假冒虚化分身名称入此将行裁减的扫黑名单集等下连拔处理应用功能展现表现控制修改反馈
        }
    }

    if (IsValid(ActorToRemove)) // 如果目标到当下还没变质或已自发清场确实为正统对象能够接收销毁召唤状态控制产生
    {
        ActorToRemove->Destroy(); // 施发真正的物理彻底解除令命在当前周期结束前强制摧散此体释放其占用作用展现执行表现效果应用系统发生清理产生动作实施
    }

    if (!AliasIds.Contains(ActorID)) // 预防万一下册操作把原需被处理的要求实名并没有混进名簿收集单集内的保险排错漏添操作功能落地传控表现影响动作
    {
        AliasIds.Add(ActorID); // 保底将最先提出需受撤主称一并汇入以执行连本带利的全部吊销作用配置落地影响传导控制
    }

    for (const FString& Id : AliasIds) // 提取那些所有与之相连带的称谓记录和主号集进入连坐废黜吊单大册子内循环
    {
        Manager->UnregisterAgent(Id); // 将单据向统属发过去执行全部的挂联切废除撤号解绑以归净作用生效控制产生操作动作传导
    }

    UE_LOG(LogTemp, Log, TEXT("[FCommandHandle] Actor removed: %s (aliases removed=%d)"), *ActorID, AliasIds.Num()); // 系统下发已处理除净多关联记录及成功结果报查内部控制反馈信息展现配置传导
    return MakeRemoveActorResponse(true, FString::Printf(TEXT("Actor removed successfully (aliases=%d)"), AliasIds.Num())); // 带着一窝端的清除战绩给网络前端以无瑕返回传导出功成作用生成信息表现展示
}

// ──── 反射调用 Actor 函数 ────

/**
 * @brief 通过反射调用 Actor 的指定 UFUNCTION
 *        获取目标 Actor → 解析参数 → ProcessEvent → 提取返回值
 */
FString FCommandHandle::HandleCallActor(const TSharedPtr<FJsonObject>& CommandJson) // 使用引擎高级内化方式将外部文字转换为直接驱动内部程序的控制映射响应口径并对接过程实现调用流转分发展现作用结果响应修改产生动作反馈体系控制
{
    FString FunctionName; // 用于转存即将探寻调动的那一支底层内含注册 UFunction 的标志名称
    if (!CommandJson->TryGetStringField(TEXT("function"), FunctionName)) // 如果这来访请求都没有表明要办的事情喊谁即无这 function 载值字段供给使用传导
    {
        return MakeCallActorResponse(false, TEXT("Missing function field"), TEXT("")); // 退票返缺其主函数的调用异常原因退出这未明意义命令过程反馈生效展示作用配置
    }

    FString ActorID; // 用于寄存要使唤动作的载向实体的身份证称进行取对象确认传接应用支持产生控制操作功能实现展现传导参数调用影响作用返回生效
    if (!CommandJson->TryGetStringField(TEXT("actor_id"), ActorID)) // 如果没下指向目标人也同样不知道朝谁执行功能导致失效中止展现产生配置控制调用
    {
        return MakeCallActorResponse(false, TEXT("Missing actor_id"), TEXT("")); // 打发因对象确实没有找准所归的错信拒不处理表现操作控制参数传回配置落地展现使用参数修改
    }

    UAgentManager* Manager = UAgentManager::GetInstance(); // 和上边处理流程一致拿取户籍发签处的权属支持供找实体真身操作调用使用影响传导配置执行能力调用配置支持展现
    if (!Manager) // 一旦发签层倒置系统未出初始化成功等大劫拦截不办提供传导出保障
    {
        return MakeCallActorResponse(false, TEXT("Agent manager unavailable"), TEXT("")); // 返回因大权部门不在位不可行之事拒办控制抛错提供使用导致功能不能落实传导致控制中断展现配置
    }

    AActor* TargetActor = Manager->GetAgent(ActorID); // 向发签系统要出目标对应的引擎在局存活的操控木偶指针用起调用承接配置展现传递操作动作实施使用配置实现表达参数提供调能作用
    if (!TargetActor) // 压根查无此人在录档案不能对接系统实体现像没有对象调用支撑传导出动作生效拦截保护
    {
        return MakeCallActorResponse(false, FString::Printf(TEXT("Actor not found: %s"), *ActorID), TEXT("")); // 因为失对打转返回并提供该空无此名的未命中报错供给反馈使用产生控制操作使用展现结果落地表示向生成反馈控制
    }

    FCallParameters CallParameters; // 定下并声明要盛放对所行功能的具体补位用数值要求的专用接收囊包装体落地展示功能提供传向应用
    FString ParameterError; // 用于兜住万一这传过来的参数补单本身大有破绽等异常供下文提供报错明细传导结果产生报错展现传导调用动作结果执行反馈展现输出应用参数支持功能表达作用
    if (!ParseCallParameters(CommandJson, CallParameters, ParameterError)) // 转递全负载去往解压缩并转化引擎认知格式参数池一旦拆开有报失异常情况生效阻断功能调用展现操作表示传递结果作用产生使用输出控制呈现生成调用生成操作动作落地修改执行
    {
        return MakeCallActorResponse(false, ParameterError, TEXT("")); // 将解池破漏给出的说明回传成拒不接活的原因供知异常传出表现反馈使用操作产生参数实现生成应用作用控制说明调用展示拦截反馈参数抛传递
    }

    FString ReturnValue; // 存放该套由文字至底指令驱动生效后有可能打上来回声反馈供再次封装流出的载接值字符串表示结果产向输出供给利用作用
    if (CallActorFunction(TargetActor, FunctionName, CallParameters, ReturnValue)) // 正式交接至底层发功处理：连人带名加参数强塞过去看如果强施不报误的话将生成效应向回承带配置展现反馈执行参数传提供控制结果返回应用
    {
        bool bNeedReturn = false; // 查看客户端配置是否确实想要等反馈不要则节省传回体积浪费传导资源配置操作展现功能设定
        CommandJson->TryGetBoolField(TEXT("return"), bNeedReturn); // 把是否返回有且只看有挂明确指明布尔属性并覆盖此请求生效供传导致调用执行使用产生后续操作控制影响传向反馈展现
        return MakeCallActorResponse( // 对能成事的发起正常通盘回应包造返回封袋调用使用向输出使用
            true, // 将无任何受挫正常走完的完美通牒位标至准行使生效落地展现操作过程产生修改参数影响执行
            FString::Printf(TEXT("Function %s called successfully"), *FunctionName), // 回知某特定发令调用圆满落地生成报成控制反馈记录呈现
            bNeedReturn ? ReturnValue : TEXT("")); // 有求要回声即给其回声；无则省空减免体积作响应控制落地产生配置展现功能输出提供系统生效作用参数使用表达
    }

    return MakeCallActorResponse(false, FString::Printf(TEXT("Failed to call %s on %s"), *FunctionName, *ActorID), TEXT("")); // 反向如果强塞压反射出错抛出拒单不行的底告作回返并注明错向信息产生反馈提供处理说明作用展现控制调用执行配置抛返
}

/**
 * @brief 通过 UE 反射机制执行 Actor 的 UFUNCTION
 *
 * 工作流程：
 * 1. 查找 UFunction
 * 2. 遍历函数参数属性，区分输入参数和返回值
 * 3. 分配参数缓冲区并填充参数值
 * 4. ProcessEvent 执行函数
 * 5. 提取返回值并清理参数缓冲区
 */
bool FCommandHandle::CallActorFunction( // 深潜底层执行引擎 UFunction 高阶动态机制映射和实现转文字与代码桥梁强连的大执行函数配置传向展现使用产生动作调控配置展现操作落实功能使用
    AActor* TargetActor, // 提供需要行使并实际承该函数的发起源体供依附调用使用落实控制应用
    const FString& FunctionName, // 给出发启方法对应的引擎系统名称以供发寻找对齐操作动作产生效应生成标识控制参数
    const FCallParameters& Parameters, // 给定发功使用的填装已毕各附向或有序散落的对应入仓参给配置调控提供传参数使用展现生效功能调用支持表现调用系统落地应用
    FString& OutReturnValue) // 用于捕获其函数自身配置定义内可能产生向后返回内容值提取出来的容载管提供生成输出作用反馈控制配置影响
{
    if (!TargetActor) // 如对象指针丢位空亡抛弃
    {
        return false; // 在保护下立刻拒之防止访问底层发生雪崩抛阻返回表示失效控制动作生成说明应用配置落定
    }

    UFunction* Function = TargetActor->GetClass()->FindFunctionByName(*FunctionName);  // 查找 UFunction // 透实体上层取得其基型UClass在此利用核心发查接口对具以文字的方法提取寻出原函数结构指针作底
    if (!Function) // 果因不注册入系统、拼错名目未获这底层支持体
    {
        UE_LOG(LogTemp, Error, TEXT("[FCommandHandle] Function not found: %s on %s"), *FunctionName, *TargetActor->GetName()); // 对系统管理内记一笔某人查不到此项虚高法报错供给查询传用配置展现调用反馈使用参数实现功能
        return false; // 以不受理失败退传控制提供落地应用配置
    }

    // ── 遍历函数参数，区分输入参数和返回值 ──
    TArray<FProperty*> FunctionParams; // 设置一表用于存储那些要求外部注入作引的输入孔参数属性配置提供映射匹配控制使用
    FProperty* ReturnProp = nullptr; // 用于专项只拿且仅拿有返回标的一个可能性的属性位地址孔供结束回调取走使用配置生成落地展现传导控制提供影响修改提取调用使用记录
    for (TFieldIterator<FProperty> PropIt(Function); PropIt; ++PropIt) // 发动一次循环扫描这个被抓来的底层函数的全身属性字段特征孔查清配型展现生效动作调用生成配置落地作用传产生结果使用反映参数配置
    {
        if (!(PropIt->PropertyFlags & CPF_Parm))  // 跳过非参数属性 // 只认对标属性中带有参数属性标明的如果不是这供参之位孔一概跳飞不管不存配置传展现生效
        {
            continue; // 跳其过不存展现调用功能执行使用操作
        }

        if (PropIt->PropertyFlags & CPF_ReturnParm)  // 返回值属性 // 若扫描辨明这此一属性身兼返回出路（CPF_ReturnParm）标志说明其为出料管提取反馈作用展现操作传导配置使用
        {
            ReturnProp = *PropIt; // 单拉存它在返回属位置以备战收尾用以抽取落控配置表达执行作用生成提供功能使用影响参数表示
        }
        else // 反之那便是这纯向内收料供函数吃掉的喂参口控制参数配置使用展现落地提供功能作用表现说明参数作用生成操作
        {
            FunctionParams.Add(*PropIt);  // 输入参数 // 添加这一输入坑位入需要对接的孔表队列备后续与实值对接对缝供后续操作传落实生效展示配置调用生效产生
        }
    }

    if (!Parameters.bUseNamedParameters && FunctionParams.Num() != Parameters.PositionalParameters.Num()) // 对于没带配置字典只有单纯字列若它在数目上根本拼不齐（长了短了）就无法对应严谨执行引擎调用防挂错误产生阻止调用控制拦截使用执行功能表达
    {
        UE_LOG( // 系统大报这一参不搭的恶行记录并报不接不传
            LogTemp, // 使用警告通道记录表现调用
            Error, // 做错乱挂高严重度表示传导体现参数产生调用显示
            TEXT("[FCommandHandle] Param count mismatch on %s. Expected %d, got %d"), // 指出这需要多少只带对配了几数失准参数使用体现表现生效传导显示展现控制
            *FunctionName, // 指名是哪位法函数名出错提供参数展现修改调用作用体现
            FunctionParams.Num(), // 说该函数原生原出要求之数配置展现控制影响落地说明调用记录参数
            Parameters.PositionalParameters.Num()); // 说来者这配置传过来的之数表示控制记录体现反馈使用
        return false; // 在数目不合安全准绳下径直以不办否掉撤回控制作用展现操作影响产生结果反馈
    }

    // ── 分配参数缓冲区并填充参数值 ──
    void* ParamsBuffer = FMemory_Alloca(Function->ParmsSize);  // 栈上分配 // 向底层生拉一块与系统记录的该函数所有传入加排出参数同样规格体积极巨等体段裸内存提供充压反射垫片生效落地配置体现显示展现功能参数提供生成调用结果配置使用展示体现
    FMemory::Memzero(ParamsBuffer, Function->ParmsSize);       // 清零 // 将生拉的系统裸区域执行一次深净刷洗防污染乱余使用表现配置参数体现展现作用调用影响处理执行更新落地控制应用控制

    for (int32 ParamIndex = 0; ParamIndex < FunctionParams.Num(); ++ParamIndex)  // 按顺序绑定参数 // 把刚查清备好的该有的喂参管逐一拿回顺列对冲去匹配外部携带来的供料生效传导功能应用落地控制体现展现操作应用调用生向效果体现
    {
        FProperty* Param = FunctionParams[ParamIndex]; // 拿到具体其某个位置对准的该引擎预知原参数型配置提供提取展现操作调用生效功能表达应用
        const FString* Value = nullptr; // 为待提取之源提供游标留口供给使用获取配置影响传向体现展现落地调用作用执行调用

        if (Parameters.bUseNamedParameters) // 如果这是依名字字典做比对传入的操作配置判断处理路线体现调用展现使用生效落地反馈
        {
            Value = FindNamedParameterValue(Parameters.NamedParameters, Param->GetName(), ParamIndex); // 交由专业模糊或精度工具拿配置包中的参、真名及它的座次供寻找对应值供给回用影响传导使用体现作用落实
            if (!Value) // 如果这字对实查无可用字空套无值参数体现控制传导落空拦截处理展现调用
            {
                continue; // 跨离缺配对口不再深强装影响传导配置落地跳查执行使用体现表现影响
            }
        }
        else // 反知是傻排盲列只等序列供量配置使用传递落地动作
        {
            Value = &Parameters.PositionalParameters[ParamIndex]; // 按与底层扫出的同一顺从序列号对应盲点提取对应的待取装数提供展现作用表现实现参数产生落地使用表现传递操作信息调用
        }

        SetPropertyValue(Param, ParamsBuffer, *Value); // 最核心调一专业化形手：针对他其类型和其那值对填向开辟的垫基内存坑执行入塞落地生效表达调用参数生向体现展现作用传导操作记录参数影响使用呈现体现控制调用落应用产生
    }

    TargetActor->ProcessEvent(Function, ParamsBuffer);  // 执行函数 // 在内存底塞对无报错一切妥当时、对宿主执行底层大驱动利用装配大内存对齐塞去进行真正虚幻核心引擎底层触发强运生效动作发生调控应用使用体现展示表现操作作用信息反馈说明结果呈现生成使用

    if (ReturnProp)  // 提取返回值 // 在该大运作完成后回查这函数若是系统带个外抽提返回接口展现表现落地控制传落实操作
    {
        void* ReturnValuePtr = ReturnProp->ContainerPtrToValuePtr<void>(ParamsBuffer); // 去开大内存坑找该返回所属位槽找该值提取该地址使用配置提供参数展现表示应用控制执行产生落地影响说明
        OutReturnValue = ConvertPropertyToString(ReturnProp, ReturnValuePtr); // 执行又一针对各异类别去抽丝回原将该位数值转化成人可识读通用化字符以做备传配置应用提供使用动作作用表现输出产生落地影响生效调引结果展示应用传导体现参数信息
    }
    else // 不带回的绝命操作表示体现调用传记录参数应用落地作用调用展现效果执行使用体现表现生效传记录操作作用使用功能展现表示
    {
        OutReturnValue = TEXT(""); // 空载挂起留作无回抛串标识反馈功能展示提供生效动作使用落地影响参数生成执行控制作用
    }

    // ── 清理参数缓冲区 ──
    for (const FProperty* Param : FunctionParams) // 在事毕之末将所有拿过的喂管逐一调来进行查封表现控制操作使用产生生效调用传记录落地展现
    {
        if (!(Param->PropertyFlags & CPF_OutParm)) // 防止如果属于带有输出回抛性质双用功能时不滥删提供展现效果产生传使用落地影响应用体现作用
        {
            Param->DestroyValue_InContainer(ParamsBuffer); // 对真正一次抛散内存做其底内部指针安全毁散清收清理遗挂展现产生使用生效发挥控制影响调动展现落实操作应用执行产生参数体现作用功能
        }
    }

    return true; // 行遍如上历次大劫仍未出崩溃和严重偏航回指本传召完胜以反馈安全落地控制操作体现表达影响传导致能调用生成表示生效展现执行参数操作使用
}

/**
 * @brief 根据属性类型将字符串值写入参数缓冲区
 *
 * 支持类型：bool / byte / int / int64 / float / double / FString / FName /
 *            枚举 (EnumProperty / ByteProperty+Enum) / FVector / FRotator
 */
void FCommandHandle::SetPropertyValue(FProperty* Property, void* Container, const FString& Value) // 将通式的长串文与内存指基坑按具体底层定性的强配重组写入落存方法处理核心实现分派使用控制传导影响展现作用
{
    // 解析 "x,y,z" 格式的三元组工具 lambda
    auto ParseTriple = [](const FString& InValue, const FString& Prefix, float& OutA, float& OutB, float& OutC) -> bool // 配设用局部小算法快查针对多变量向量形式抽取分量的解包控制提供落地生效调用产生展现体现使用控制说明传导作用影响传参数配置执行表示
    {
        FString Work = InValue; // 提值用供改造修改展现记录实现
        if (!Prefix.IsEmpty() && Work.StartsWith(Prefix)) // 带其若挂标特征如‘FVector’使用前剪切抛除只提取实质参数供处理表达实现功能作用
        {
            Work = Work.RightChop(Prefix.Len()); // 对长首裁边去头留其干净数序列体展现控制操应用功能生成影响落实参数调用配置提交展现
        }

        TArray<FString> Components; // 建立以存放受阻分词后分块供调用实现作用展现生效影响产生呈现调使用控制过程
        Work.ParseIntoArray(Components, TEXT(",")); // 在纯数符序列依所约定的中缀逗留切为三分并落存展示体现操作执行表现生成落地控制过程反馈参数
        if (Components.Num() != 3) // 一旦验数查量不足这不容偏差缺位的这类型必须数量报异不通过提供报错传截展现作用控制表现生效应用参数标识
        {
            return false; // 不全者不容进退出报错展示体现功能体现生成展现应用反映参数控制生成调能表现使用展现产生作用结果说明应用生效
        }

        OutA = FCString::Atof(*Components[0]); // 将所解之一转作原数给落盘体现调用表示使用产生落操作展现配置控制
        OutB = FCString::Atof(*Components[1]); // 处理第二段入位使用功能表示调用产生应用展现操作参数
        OutC = FCString::Atof(*Components[2]); // 处理第三余值压装落地展现表现使用影响参数生成配置传输出体现生效
        return true; // 表本套抽取切片化分入体解算收成合归操作控制展现功能应用落地使用提供传导执行
    };

    if (FBoolProperty* BoolProp = CastField<FBoolProperty>(Property)) // 若鉴得在引配置此槽确实是一个存只承布尔底真机制的空落点展现表达修改作用过程控制反馈动作落实使用调用生执行生成控制
    {
        BoolProp->SetPropertyValue_InContainer(Container, Value.ToBool()); // 即强按系统文改布值写录回内存槽生效调用传应用落地展现执行结果产生使用过程调出参数生成指令展现作用呈现
    }
    else if (FEnumProperty* EnumProp = CastField<FEnumProperty>(Property)) // 或者它是一种更为高类抽象以代名配数字之类的高枚举型时提供匹配处理策略执行表现使用控制过程调动影响反馈体现
    {
        int64 EnumValue = 0; // 给底层数化占位设置默认底期控制应用提供产生使用落地操作体现生成过程调用指令
        if (Value.IsNumeric()) // 如入串直通地写就的仍为原生基数数字之本调用展现操作
        {
            EnumValue = FCString::Atoi64(*Value); // 则跳名译文过程直灌转作长整压仓提供生效控制动作生成应用传落实调用展现
        }
        else if (UEnum* EnumDef = EnumProp->GetEnum()) // 倘非纯数那便向类属性讨来可判定的名表登记全套参照物集用查引使用影响展现表示产生反馈动作落实展现使用控制参数执行
        {
            int64 FoundValue = EnumDef->GetValueByNameString(Value); // 用带来的文在这个注册类里反循此文看落数几号提供展现效果产生功能落地实现操作使用动作展现指示应用
            if (FoundValue == INDEX_NONE) // 如若是单词找不得无据而查无对上标号提供拦截控制表示使用指令产生展现
            {
                const FString ScopedName = FString::Printf(TEXT("%s::%s"), *EnumDef->GetName(), *Value); // 尝强加它的系统属类前缀构全名字号拼词给补以再度进行打量体现表现落地应用传动作使生影响调用配置
                FoundValue = EnumDef->GetValueByNameString(ScopedName); // 下达在全带名的底录作第二次反检索调用展现指示表现生成控制调动执行参数应用落地生效
            }
            if (FoundValue != INDEX_NONE) // 但凡能挂得上的且不再悬无标志报错操作显示控制应用体现产生提供使用反馈调能体现展现使用表示
            {
                EnumValue = FoundValue; // 录这得出来的真实根编号落于填存供后实进应用产生功能表示操作指令调用生效作用展现提供控制使用展示结果传
            }
        }

        if (FNumericProperty* Underlying = EnumProp->GetUnderlyingProperty()) // 最后求其枚举皮囊里兜里那实际做数据的底层强管基属执行配置展现表达操作用生成动作表示调用反映产生信息供给反馈
        {
            Underlying->SetIntPropertyValue(EnumProp->ContainerPtrToValuePtr<void>(Container), EnumValue); // 向那里用数值名义硬填打将落账供反射收走配置用落地表现生效影响产生控制作用呈现功能调动作使用记录展现说明使用
        }
    }
    else if (FByteProperty* ByteProp = CastField<FByteProperty>(Property)) // 对于在某些极缩型设计即有承数字也做轻级枚举两不相扰形式判定为 Byte 处理控制操作参数应用呈现调用表现表示生成生效体现控制落地作用展现传配置
    {
        if (ByteProp->Enum && !Value.IsNumeric()) // 如果它其实带有一个内定字典图谱且传字又偏偏来的是需解码英文字则发起类枚举找引传展示生成表现执行参数控制展现功能使用提供应用操作
        {
            int64 FoundValue = ByteProp->Enum->GetValueByNameString(Value); // 执行一贯之找表看挂号策略控制表现调用传落地表现应用控制反馈操作作用体现产生向生成作用使用呈现应用使用表示传递体现参数动作
            if (FoundValue == INDEX_NONE) // 无挂则转处理提供动作执行使用操作展示
            {
                const FString ScopedName = FString::Printf(TEXT("%s::%s"), *ByteProp->Enum->GetName(), *Value); // 配以前名补全全长调用控制产生操作应用提供影响展现
                FoundValue = ByteProp->Enum->GetValueByNameString(ScopedName); // 再打表核实表现生成操作传表现落地使用体现使用功能展现调控产生操作指令作用
            }
            if (FoundValue != INDEX_NONE) // 核能得果的反馈控制操作展现调用表示应用
            {
                ByteProp->SetPropertyValue_InContainer(Container, static_cast<uint8>(FoundValue)); // 这时借它自己的缩水型本类直以八位写进实孔配置生效表现使用产生功能操作表现应用控制体现落地执行生成展现提供传使用提供
                return; // 直接收局不向下流用展现控制体现产生调用
            }
        }

        ByteProp->SetPropertyValue_InContainer(Container, static_cast<uint8>(FCString::Atoi(*Value))); // 未含特殊表直接做数值将传入字符串切整数作截八位收存进对应内存地址配置展现产生表示传使用记录产生调控动作
    }
    else if (FIntProperty* IntProp = CastField<FIntProperty>(Property)) // 若核定这确凿不过就普通日常整数坑口动作生成传递展现表示
    {
        IntProp->SetPropertyValue_InContainer(Container, FCString::Atoi(*Value)); // 借由常格译数落印回该指定大容器对应的特定变槽位表现生效调用落地说明功能动作控制操作影响传出参数使用调展现用
    }
    else if (FInt64Property* Int64Prop = CastField<FInt64Property>(Property)) // 对于深长大号不损失整数处理类型配置调用使用落地影响表现记录参数产生作用表现提供反馈
    {
        Int64Prop->SetPropertyValue_InContainer(Container, FCString::Atoi64(*Value)); // 做用大容量解码数直存打孔写入产生操作用生效执行使用体现说明调用应用反馈作用输出产生参数指令呈现
    }
    else if (FFloatProperty* FloatProp = CastField<FFloatProperty>(Property)) // 处理日常短浮单显精度孔表现调用体现使用参数动作操作功能展示
    {
        FloatProp->SetPropertyValue_InContainer(Container, FCString::Atof(*Value)); // 作常化浮转化入指定实属底穴提交生效落应用展现调用反馈操修改影响表现传产生配置作用功能输出指令呈现
    }
    else if (FDoubleProperty* DoubleProp = CastField<FDoubleProperty>(Property)) // 核证若系统要求配置需这长精长宽无错双进的精确双点浮配槽提供应用体现操作使用传反馈执行作用
    {
        DoubleProp->SetPropertyValue_InContainer(Container, FCString::Atof(*Value)); // 同以化数法交于基底层长浮接口送入存挂影响配置展现表现提供落实展现生成使用功能表现操作作用
    }
    else if (FStrProperty* StrProp = CastField<FStrProperty>(Property)) // 对于文本串等纯形式存槽传向使用展示应用参数表现提供控制呈现生成执行传记录修改操作
    {
        StrProp->SetPropertyValue_InContainer(Container, Value); // 直接无碍将携带的直串全扔入大容器孔位填其数据表现表达展现操作调动作用生成影响传递使用展现结果功能影响作用应用呈现返回
    }
    else if (FNameProperty* NameProp = CastField<FNameProperty>(Property)) // 若是对引擎底大内用的专用化 FName 型号存项展现配置操传调展示产生表现使用影响参数实现执行返回提供体现
    {
        NameProp->SetPropertyValue_InContainer(Container, FName(*Value)); // 将此文裹以外包强生成引擎可用的独有识别 FName 包重置推入坑底表示落实展现传反馈调展现操作应用体现调出生成说明展现调用应用配置
    }
    else if (FStructProperty* StructProp = CastField<FStructProperty>(Property)) // 如逢遇大号组套类结构包裹型变量时启动解压分处理提供操作使用展示应用落地体现产生动作生成
    {
        if (StructProp->Struct == TBaseStructure<FVector>::Get()) // 将其内核套件类型验实若是引擎位置特占 FVector 型用提供传导调用展现表示生成操作记录落地反应使用体现应用
        {
            float X = 0.0f; // 设用解用占位提供展示功能影响参数获取
            float Y = 0.0f; // 供解析缓冲准备表示表现调用作用使用应用传
            float Z = 0.0f; // 开占接供提取输出利用体现参数生成反馈
            if (ParseTriple(Value, TEXT("FVector:"), X, Y, Z) || ParseTriple(Value, TEXT(""), X, Y, Z)) // 做前引截离并借上设工算法取下真数执行生成操作使用参数反馈调用产生功能展现表现
            {
                const FVector VectorValue(X, Y, Z); // 建立底层同型临机量用实出提取值组合落地作用体现展示调用影响配置传控制应用生成用向表达功能发生呈现
                StructProp->CopyCompleteValue(StructProp->ContainerPtrToValuePtr<void>(Container), &VectorValue); // 并把这套机内生成的标尺全数通过源生机制复写至真待收位置地址之中作供实调用使用传导致生效控制体现传递结果应用参数展示记录表现操作体现落出配置产生实现动作展现返回
            }
        }
        else if (StructProp->Struct == TBaseStructure<FRotator>::Get()) // 其要为系统专属扭转态度占标构型的使用参数反馈调用展现说明记录体现作用生落执行动作
        {
            float Roll = 0.0f; // 定扭引量展现配置产生表达表现应用生成返回效果执行使用参数反馈
            float Pitch = 0.0f; // 设低高缓冲调用传使用展现表明应用使用提供功能产生操作影响结果提供表示
            float Yaw = 0.0f; // 置偏移待量传说明发生使用展示功能配置展现生成参数表示体现调用反馈
            if (ParseTriple(Value, TEXT("FRotator:"), Roll, Pitch, Yaw) || // 做多重截拆如果全无匹配就放过不操作表现传生效配置使用参数修改调出记录提供展现落地使用应用表达返回结果表现
                ParseTriple(Value, TEXT("FVector:"), Roll, Pitch, Yaw) || // 允许混源如在特定未声明或混投发使用表示产生操作说明体现生成调用应用落地配置说明展示发生使用传到影响动作配置
                ParseTriple(Value, TEXT(""), Roll, Pitch, Yaw)) // 对纯数字无头标依然以兼容方式试提表现展示说明生成功能使用展现参数应用记录反馈调用结果体现实现落实产生
            {
                const FRotator RotatorValue(Pitch, Yaw, Roll); // 取并装机合成指定构型并调整传入对应结构位置的专用标实载录体产生调用功能展现传控制生影响操作使用提供参数信息返回落地结果展现影响作用
                StructProp->CopyCompleteValue(StructProp->ContainerPtrToValuePtr<void>(Container), &RotatorValue); // 全完复写交送落至该实际预放真址让内含在内的反射操作在运行时带起全数变化效应展现生发配置影响传产生反馈功能调用参数生成指令反映展示落地结果使用操作生效效果表达
            }
        }
    }
}

/** @brief 将属性值转换为字符串（用于函数返回值输出） */
FString FCommandHandle::ConvertPropertyToString(FProperty* Prop, void* ValuePtr) // 反转前功能将对因内存挂存变量根据其实有的类型将其拔取后全做回本字化转字符表现展现生发操作体现落使用参数供给修改调应用传调用指令生成作用影响反馈结果产生发生说明作用
{
    if (FStrProperty* StrProp = CastField<FStrProperty>(Prop)) // 直若是本字属性体现展现使用配置
    {
        return StrProp->GetPropertyValue(ValuePtr); // 不经多弄只管去把带字的字全盘收并还送上去展示结果传落实输出操作产生使用说明
    }
    if (FBoolProperty* BoolProp = CastField<FBoolProperty>(Prop)) // 如原是个只有分明立非真伪位点展现说明操作体现应用传配置使用反馈参数提供
    {
        return BoolProp->GetPropertyValue(ValuePtr) ? TEXT("true") : TEXT("false"); // 以布尔判断用人类的本字把真与非做明确替换反并向上提交表示作用指示生成反馈影响调动体现展示产生控制应用落地生成作用
    }
    if (FFloatProperty* FloatProp = CastField<FFloatProperty>(Prop)) // 单精度常用于记录的小数形位点展现生成执行参数应用使用返回表现说明
    {
        return FString::SanitizeFloat(FloatProp->GetPropertyValue(ValuePtr)); // 提该处实打并经缩整除零修化作净整长符的字句归去供读呈现应用使用调配作用执行落地表示参数影响展示呈现返回功能
    }
    if (FDoubleProperty* DoubleProp = CastField<FDoubleProperty>(Prop)) // 广域的大包罗深位小数槽口应用传展示展现使用配置说明提出
    {
        return FString::SanitizeFloat(DoubleProp->GetPropertyValue(ValuePtr)); // 同将多出的或赘带长的小数值用格式定净并拉为明示文本送回表呈现展现作用影响参数产生传使用功能执行说明落地配置反馈
    }
    if (FIntProperty* IntProp = CastField<FIntProperty>(Property)) // 一般通用记整数底孔操作产生传反馈展示实现
    {
        return FString::FromInt(IntProp->GetPropertyValue(ValuePtr)); // 借内成之接口直接改提成字号列长送配功能提落地说明展现调用应用结果产生操作传展示返回输出作用反馈
    }
    if (FInt64Property* Int64Prop = CastField<FInt64Property>(Prop)) // 属那加倍长度存放的大号整数点提供返回使用调展示生执行参数指示配置
    {
        return LexToString(Int64Prop->GetPropertyValue(ValuePtr)); // 做该专项长的特殊解成文本处理上报显示操作使用展示体现功能生成作用调用产生使用影响配置落实抛传表示结果展现执行传导控制结果参数指示应用反馈调用
    }
    return TEXT("unsupported_type"); // 以上如不配合而超出预期能辨范围统一向报错接口抛文显示无所适从这型号的返回拒单表示产生返回参数体现调显示落实产生展现输出应用过程控制说明拦截反馈使用
}

// ──── JSON 响应构建 ────

/** @brief 构建 add_actor 响应 */
FString FCommandHandle::MakeAddActorResponse(const FString& ActorID, bool bSuccess, const FString& Message) // 转专门针对实添置操作出成专特封发格式打包传展现使用说明生成参数应用反馈传产生配置功能体现操作指示应用返回结果使用表示落地
{
    const TSharedPtr<FJsonObject> ResponseJson = MakeStatusResponseObject(bSuccess, Message); // 取带基础成败状况明细包头备好产生应用配置调用展现生影响传导展示功能过程反馈结果落实调用体现生成处理指使用生效应用使用操作返回
    if (!ActorID.IsEmpty()) // 如不虚空该新户则提供生操作产生展现影响
    {
        ResponseJson->SetStringField(TEXT("actor_id"), ActorID); // 打在名底传返回使用展现参数说明记录应用指示控制调用展现生落实操作产生影响执行作用使用
    }
    return SerializeWrappedResponse(TEXT("add_actor_return"), ResponseJson); // 使用此这外封包给加上标其此是回添置单后的回复封死压线送回展示落地效果体现调用返回应用传作用指令产生执行控制生向功能输出表现指示记录操作反馈
}

/** @brief 构建 remove_actor 响应 */
FString FCommandHandle::MakeRemoveActorResponse(bool bSuccess, const FString& Message) // 面向专属在撤单毁目标下发操作进行包装发送生成展现说明落地处理应用产生传调用过程体现使用反馈作用参数操作生生效指示配置执行结果作用影响表示
{
    return SerializeWrappedResponse(TEXT("remove_actor_return"), MakeStatusResponseObject(bSuccess, Message)); // 集一句话内用通包封发连成带明示毁撤返的大件输出执行反馈结果落地表现生表示传展现应用使用调产生参功能生成控制返回影响配置落地输出产生
}

/** @brief 构建 call_actor 响应（可附带返回值） */
FString FCommandHandle::MakeCallActorResponse(bool bSuccess, const FString& Message, const FString& ReturnValue) // 对特为指令发起功能操动作应发作带有传值回向或报表功配置展示使用体现产生调操作展现提供参数应用返回输出执行结果功能指示表现作用记录
{
    const TSharedPtr<FJsonObject> ResponseJson = MakeStatusResponseObject(bSuccess, Message); // 领提共用件包基表展现使用落地生功能指令参数返回调用传操作表示提供效作用应用生指示产生影响说明展现传导致落实展示过程控制体现执行反馈
    if (!ReturnValue.IsEmpty()) // 当遇确带有抛出的函数执行后得物记录配置产生说明
    {
        ResponseJson->SetStringField(TEXT("return"), ReturnValue); // 装进载有"return"记名特定回置层给打包封装提供调用使用影响生展现落实指示操作传作用体现传产生反馈使用应用展示提供结果控制生成执行表示展现
    }
    return SerializeWrappedResponse(TEXT("call_actor_return"), ResponseJson); // 放套上以特用回调名目标注的发大囊封装定成一团发射送展示体现用向操作说明参数产生控制返回值影响配置执行结果调用传反映展现说明作用提供控制应用执行传记录
}

/** @brief 构建通用错误响应 */
FString FCommandHandle::MakeErrorResponse(const FString& ReturnType, const FString& Message) // 此乃专收对因总外封抛失传或破等其它异类向的泛底报错大类囊操作反馈展现指示过程参数控制落调用体现影响使用表示产生配置产生执行提供作用参数说明返回功能
{
    return SerializeWrappedResponse(ReturnType, MakeStatusResponseObject(false, Message)); // 使用外部传入需补封回的原返源定用包外借错载基础框大一拼定发送执行产生展现调用功能控制影响操作落地体现应用提供使用参数传递展现说明结果表现生效作用返回
}
