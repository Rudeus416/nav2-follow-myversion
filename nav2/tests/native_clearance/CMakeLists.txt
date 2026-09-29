# 【内容标注】用途：导航离线回归：CMakeLists。
# 对应用户需求：R13 R14 R22（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：对应模块的输入边界、故障或几何场景验证；测试名描述具体断言，不作为实车通过证明。
# 本次仅加注释；需求关联不是精确创建/提交记录。
cmake_minimum_required(VERSION 3.16)
project(nav2_clearance_regression)
# 【职责 / R13 R14 R22】测试依赖：独立编译整车净空回归，不启动 Nav2 或底盘。
find_package(ament_cmake REQUIRED)
find_package(nav2_smac_planner REQUIRED)
find_package(ompl REQUIRED)
# 【职责 / R13 R14 R22】测试目标：复现整车、缓冲和转弯净空边界。
add_executable(clearance regression.cpp)
target_include_directories(clearance PRIVATE ${OMPL_INCLUDE_DIRS})
ament_target_dependencies(clearance nav2_smac_planner)
target_link_libraries(clearance ${OMPL_LIBRARIES})
