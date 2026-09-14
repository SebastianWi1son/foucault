cmake_minimum_required(VERSION 3.21)
project(foucault LANGUAGES CXX)

# foucault — 通用可移植姿态解算核心
# PC 可测优先：先桌面验证数学/算法，再移植 H7（platforms/ 后续接入）
# 零依赖（仅标准库头文件）；C++17；-Wall -Wextra -Werror
# target 三件套：foucault_core（header-only 接口库）/ test_math（锚点测试）/ example_ahrs（批次 2 起）

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# ── 核心库：头文件部分，用 INTERFACE 库（无编译产物，只传播头文件路径与编译选项）──
# 消费方： target_link_libraries(<你的 target> PRIVATE foucault_solver)
add_library(foucault_core INTERFACE)
target_include_directories(foucault_core INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/core)
target_compile_options(foucault_core INTERFACE -Wall -Wextra -Werror)

# ── 求解器实现（有 .cpp，**必须**进一个 STATIC 库）──
# 2026-09-14 补：原先 mahony.cpp 只被直接写进各个 test 可执行文件，
# foucault_core 里没有任何 .o —— 外部工程 link foucault_core 会报一堆
# "undefined reference to foucault::solver::Mahony::..."。
# 一个库要能被别人用，必须导出一个【完整可链接】的 target。
add_library(foucault_solver STATIC core/solver/mahony.cpp)
target_link_libraries(foucault_solver PUBLIC foucault_core)
target_compile_options(foucault_solver PRIVATE -Wall -Wextra -Werror)

# ── 被别的工程当子项目引入时（add_subdirectory / FetchContent），只提供库 ──
#    否则固件工程会连着生成 test_math / replay_nav2 / ... 一堆宿主可执行文件，
#    交叉编译时直接失败。（与 lunokhod 各组件同一套守卫）
if(NOT PROJECT_IS_TOP_LEVEL)
    message(STATUS "foucault: 作为子项目引入 -> 只提供库 foucault_core / foucault_solver")
    return()
endif()
message(STATUS "foucault: 顶层构建 -> 生成测试 / 回放工具")

# ── 锚点测试（行为不变式；ASan/UBSan 加持）──
enable_testing()
add_executable(test_math tests/unit/test_math.cpp)
target_link_libraries(test_math PRIVATE foucault_core)
target_compile_options(test_math PRIVATE -fsanitize=address,undefined)
target_link_options(test_math PRIVATE -fsanitize=address,undefined)
add_test(NAME test_math COMMAND test_math)

# ── core/math 独立审计测试（AI 编写 2026-09-12）──
# 独立性：自写参考实现（4x4 矩阵 Hamilton 积 / Rodrigues 旋转）+ Python 生成金标数据
# （生成器 tests/tools/gen_math_golden.py；头文件 tests/unit/math_golden.hpp 自动生成）
# 盯住 core/math 转归档状态：改 math 目录时此 target 必须全绿
add_executable(test_math_audit tests/unit/test_math_audit.cpp)
target_link_libraries(test_math_audit PRIVATE foucault_core)
target_include_directories(test_math_audit PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests/unit)
target_compile_options(test_math_audit PRIVATE -fsanitize=address,undefined)
target_link_options(test_math_audit PRIVATE -fsanitize=address,undefined)
add_test(NAME test_math_audit COMMAND test_math_audit)

# ── 批次 2：Mahony 求解器（测试 AI 已写好；core/solver/ 抄录完成后取消注释即转绿）──
add_executable(test_mahony tests/unit/test_mahony.cpp core/solver/mahony.cpp)
target_link_libraries(test_mahony PRIVATE foucault_core)
target_compile_options(test_mahony PRIVATE -fsanitize=address,undefined)
target_link_options(test_mahony PRIVATE -fsanitize=address,undefined)
add_test(NAME test_mahony COMMAND test_mahony)

# ── 批次 3：门面测试 + NAV2 回放 ──
add_executable(test_estimator tests/unit/test_estimator.cpp core/solver/mahony.cpp)
target_link_libraries(test_estimator PRIVATE foucault_core)
target_compile_options(test_estimator PRIVATE -fsanitize=address,undefined)
target_link_options(test_estimator PRIVATE -fsanitize=address,undefined)
add_test(NAME test_estimator COMMAND test_estimator)

# 回放工具（AI 编写）：真实数据集回归，验收 roll/pitch RMSE < 5°
add_executable(replay_nav2 host/replay/replay_nav2.cpp core/solver/mahony.cpp)
target_link_libraries(replay_nav2 PRIVATE foucault_core)
target_compile_options(replay_nav2 PRIVATE -Wall -Wextra -Werror)
add_test(NAME replay_nav2 COMMAND replay_nav2
        ${CMAKE_CURRENT_SOURCE_DIR}/data/NAV2_data.bin)

# ── 示例（批次 2 起：mahony 桌面回放 / NAV2 数据集对比；届时取消注释）──
# add_executable(example_ahrs examples/example_ahrs.cpp)
# target_link_libraries(example_ahrs PRIVATE foucault_core)
