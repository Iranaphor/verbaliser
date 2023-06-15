cmake_minimum_required(VERSION 2.8.3)
project(verbaliser)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


foreach (dir scripts)
    install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}/)
endforeach(dir)
