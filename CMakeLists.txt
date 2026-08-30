cmake_minimum_required(VERSION 3.16)
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

# ── 核心库：当前为 header-only，用 INTERFACE 库（无编译产物，只传播头文件路径与编译选项）──
add_library(foucault_core INTERFACE)
target_include_directories(foucault_core INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/core)
target_compile_options(foucault_core INTERFACE -Wall -Wextra -Werror)

# ── 锚点测试（行为不变式；ASan/UBSan 加持）──
enable_testing()
add_executable(test_math tests/unit/test_math.cpp)
target_link_libraries(test_math PRIVATE foucault_core)
target_compile_options(test_math PRIVATE -fsanitize=address,undefined)
target_link_options(test_math PRIVATE -fsanitize=address,undefined)
add_test(NAME test_math COMMAND test_math)

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
