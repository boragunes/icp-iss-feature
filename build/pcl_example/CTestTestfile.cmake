# CMake generated Testfile for 
# Source directory: /home/ws/src/pcl_example
# Build directory: /home/ws/src/pcl_example/build/pcl_example
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_pcl_example_launch.testing.py "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ws/src/pcl_example/build/pcl_example/test_results/pcl_example/test_pcl_example_launch.testing.py.xunit.xml" "--package-name" "pcl_example" "--output-file" "/home/ws/src/pcl_example/build/pcl_example/launch_test/test_pcl_example_launch.testing.py.txt" "--command" "/usr/bin/python3" "-m" "launch_testing.launch_test" "/home/ws/src/pcl_example/test/pcl_example_launch.testing.py" "--junit-xml=/home/ws/src/pcl_example/build/pcl_example/test_results/pcl_example/test_pcl_example_launch.testing.py.xunit.xml" "--package-name=pcl_example")
set_tests_properties(test_pcl_example_launch.testing.py PROPERTIES  LABELS "launch_test" TIMEOUT "60" WORKING_DIRECTORY "/home/ws/src/pcl_example/build/pcl_example" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/launch_testing_ament_cmake/cmake/add_launch_test.cmake;131;ament_add_test;/home/ws/src/pcl_example/CMakeLists.txt;87;add_launch_test;/home/ws/src/pcl_example/CMakeLists.txt;0;")
