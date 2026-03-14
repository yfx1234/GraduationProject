// 解释：引入当前实现文件对应的头文件 `GraduationProject.h`，使实现部分能够看到类和函数声明。
#include "GraduationProject.h"
// 解释：引入 `ModuleManager.h`，为当前文件补充所依赖的类型、函数或接口声明。
#include "Modules/ModuleManager.h"

/**
 * @brief GraduationProject 主模块实现
 * 使用 Unreal 默认模块实现承载工程生命周期，当前不额外注入自定义启动逻辑。
 */
// 解释：调用 `IMPLEMENT_PRIMARY_GAME_MODULE` 执行当前步骤需要的功能逻辑。
IMPLEMENT_PRIMARY_GAME_MODULE(FDefaultGameModuleImpl, GraduationProject, "GraduationProject");
