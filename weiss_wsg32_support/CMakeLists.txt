cmake_minimum_required(VERSION 2.8.3)
project(weiss_wsg32_support)


find_package(catkin REQUIRED)


catkin_package()


install(DIRECTORY launch meshes urdf
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
