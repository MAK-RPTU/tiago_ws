cmake_minimum_required(VERSION 3.5)
project(tiago_2dnav)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(pal_module_cmake_FOUND)
pal_register_modules(module/10_localization.yaml
                    module/10_navigation.yaml
                    module/10_slam.yaml)
elseif(DEFINED ENV{PAL_DISTRO})
  message(FATAL_ERROR "The pal_module_cmake package was not found. Modules not registered.")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Include the maps directory in the installation
install(DIRECTORY pal_maps
        DESTINATION share/${PROJECT_NAME})

ament_auto_package(INSTALL_TO_SHARE config launch)
