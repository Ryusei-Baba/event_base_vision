cmake_minimum_required(VERSION 3.8)
project(event_base_vision)

# 必要な依存関係
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# 実行可能ファイルを追加
add_executable(event_base_vision_node src/event_base_vision_node.cpp)

# 依存関係をリンク
ament_target_dependencies(event_base_vision_node rclcpp sensor_msgs OpenCV cv_bridge)

# インクルードディレクトリを設定
target_include_directories(event_base_vision_node PRIVATE include)

# インストール設定
install(TARGETS event_base_vision_node
        RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
        DESTINATION include/)

ament_package()
