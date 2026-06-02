add_library(armor_solver OBJECT
  armor__solver.cpp
)

target_include_directories(armor_solver PUBLIC
  ${PROJECT_SOURCE_DIR}/armor__solver__
)

target_link_libraries(armor_solver PUBLIC
  ${OpenCV_LIBS}
  Eigen3::Eigen
  yaml-cpp
)