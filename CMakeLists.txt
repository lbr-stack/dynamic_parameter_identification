cmake_minimum_required(VERSION 3.16.3)
project(gravity_compensation)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(rclpy REQUIRED)
# find_package(lbr_fri_msgs REQUIRED)

install(
  PROGRAMS
    src/TrajGeneration.py
    src/trajsimulation.py
    src/ground_truth_gen_sim.py
    src/regression.py
    src/IDmodel.py
    src/CollisionCheck.py
    src/admittance_test.py
    src/convexhallExtraction.py
    src/test_model.py
    src/MetaGen.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY src/test src/resources
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
