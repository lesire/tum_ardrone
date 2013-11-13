# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(tum_ardrone)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
	roscpp roslib tf dynamic_reconfigure cv_bridge
	sensor_msgs std_srvs std_msgs geometry_msgs message_generation
	genmsg actionlib_msgs actionlib
	ardrone_autonomy)

# System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

# Declare ROS messages and services

add_message_files(
	DIRECTORY msg
	FILES
	filter_state.msg
)

add_service_files(
	DIRECTORY srv
	FILES
	SetReference.srv
	SetMaxControl.srv
	SetInitialReachDistance.srv
	SetStayWithinDist.srv
	SetStayTime.srv
)

add_action_files(
	DIRECTORY action
	FILES
	AutoInit.action
	Empty.action
	Move.action
)

generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

generate_dynamic_reconfigure_options(
	cfg/AutopilotParams.cfg
	cfg/GUIParams.cfg
	cfg/StateestimationParams.cfg
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
	CATKIN_DEPENDS ardrone_autonomy dynamic_reconfigure roscpp message_runtime actionlib_msgs actionlib
	DEPENDS qt4 libblas-dev liblapack-dev
)

# --------------------------- 3rd party ---------------------------------------------
add_custom_target(libcvd
	COMMAND make
	WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/thirdparty/libcvd
	DEPENDS libgvars3)

add_custom_target(libgvars3
	COMMAND make
	WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/thirdparty/gvars3)

# --------------------------- stateestimation & PTAM sources --------------------------------
# set header ans source files
set(STATEESTIMATION_SOURCE_FILES       
	src/stateestimation/GLWindow2.cc
	src/stateestimation/GLWindowMenu.cc  
	src/stateestimation/main_stateestimation.cpp
	src/stateestimation/DroneKalmanFilter.cpp
	src/stateestimation/Predictor.cpp
	src/stateestimation/PTAMWrapper.cpp
	src/stateestimation/MapView.cpp
	src/stateestimation/EstimationNode.cpp
  	src/stateestimation/PTAM/ATANCamera.cc
  	src/stateestimation/PTAM/Bundle.cc
  	src/stateestimation/PTAM/HomographyInit.cc
  	src/stateestimation/PTAM/KeyFrame.cc
  	src/stateestimation/PTAM/Map.cc
  	src/stateestimation/PTAM/MapMaker.cc
	src/stateestimation/PTAM/MapPoint.cc
	src/stateestimation/PTAM/MiniPatch.cc
	src/stateestimation/PTAM/PatchFinder.cc
	src/stateestimation/PTAM/Relocaliser.cc
	src/stateestimation/PTAM/ShiTomasi.cc
	src/stateestimation/PTAM/SmallBlurryImage.cc
	src/stateestimation/PTAM/Tracker.cc
)
set(STATEESTIMATION_HEADER_FILES    
	src/stateestimation/GLWindow2.h 
	src/stateestimation/GLWindowMenu.h    
	src/stateestimation/MouseKeyHandler.h  
	src/HelperFunctions.h   
	src/stateestimation/DroneKalmanFilter.h        
	src/stateestimation/Predictor.h 
	src/stateestimation/PTAMWrapper.h
	src/stateestimation/MapView.h
	src/stateestimation/EstimationNode.h
	src/stateestimation/PTAM/ATANCamera.h
	src/stateestimation/PTAM/Bundle.h
	src/stateestimation/PTAM/customFixes.h
	src/stateestimation/PTAM/HomographyInit.h
	src/stateestimation/PTAM/KeyFrame.h
	src/stateestimation/PTAM/LevelHelpers.h
	src/stateestimation/PTAM/Map.h
	src/stateestimation/PTAM/MapMaker.h
	src/stateestimation/PTAM/MapPoint.h
	src/stateestimation/PTAM/MEstimator.h
	src/stateestimation/PTAM/MiniPatch.h
	src/stateestimation/PTAM/OpenGL.h
	src/stateestimation/PTAM/PatchFinder.h
	src/stateestimation/PTAM/Relocaliser.h
	src/stateestimation/PTAM/settingsCustom.h
	src/stateestimation/PTAM/ShiTomasi.h
	src/stateestimation/PTAM/SmallBlurryImage.h
	src/stateestimation/PTAM/SmallMatrixOpts.h
	src/stateestimation/PTAM/TrackerData.h
	src/stateestimation/PTAM/Tracker.h
	src/stateestimation/PTAM/VideoSource.h
)

# ------------------------- autopilot & KI sources -----------------------------------------
# set header ans source files
set(AUTOPILOT_SOURCE_FILES         
	src/autopilot/main_autopilot.cpp  
	src/autopilot/ControlNode.cpp 
	src/autopilot/ActionControlNode.cpp
	src/autopilot/DroneController.cpp
	src/autopilot/KI/KILand.cpp
	src/autopilot/KI/KIAutoInit.cpp
	src/autopilot/KI/KIFlyTo.cpp
)
set(AUTOPILOT_HEADER_FILES        
	src/autopilot/ControlNode.h
	src/autopilot/DroneController.h
	src/autopilot/KI/KILand.h
	src/autopilot/KI/KIAutoInit.h
	src/autopilot/KI/KIFlyTo.h
	src/autopilot/KI/KIProcedure.h
)

# ---------------------------- GUI sources--------------------------------------------------
# set header ans source files
set(GUI_SOURCE_FILES         
	src/UINode/main_GUI.cpp  
	src/UINode/tum_ardrone_gui.cpp  
	src/UINode/RosThread.cpp  
	src/UINode/PingThread.cpp  
)

set(GUI_HEADER_FILES        
	src/UINode/tum_ardrone_gui.h 
	src/UINode/RosThread.h  
	src/UINode/PingThread.h  
)

###########
## BUILD ##
###########

# Set required libs and headers
include_directories(
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${PROJECT_SOURCE_DIR}/cfg/cpp
	${PROJECT_SOURCE_DIR}/thirdparty/TooN/include 
	${PROJECT_SOURCE_DIR}/thirdparty/libcvd/include 
	${PROJECT_SOURCE_DIR}/thirdparty/gvars3/include 
)  
link_directories(
	${PROJECT_SOURCE_DIR}/lib
	${PROJECT_SOURCE_DIR}/thirdparty/libcvd/lib 
	${PROJECT_SOURCE_DIR}/thirdparty/gvars3/lib 
)
set(PTAM_LIBRARIES GL glut cvd GVars3 blas lapack)
add_definitions(-DKF_REPROJ)

## StateEstimation build
add_executable(drone_stateestimation ${STATEESTIMATION_SOURCE_FILES} ${STATEESTIMATION_HEADER_FILES})
set_target_properties(drone_stateestimation PROPERTIES
	COMPILE_FLAGS "-D_LINUX -D_REENTRANT -O3 -march=nocona -msse3")
add_dependencies(drone_stateestimation StateestimationParams libcvd)
target_link_libraries(drone_stateestimation ${PTAM_LIBRARIES} ${catkin_LIBRARIES})

## Autopilot build
add_executable(drone_autopilot ${AUTOPILOT_SOURCE_FILES} ${AUTOPILOT_HEADER_FILES})
add_dependencies(drone_autopilot AutopilotParams drone_stateestimation tum_ardrone_gencpp)
target_link_libraries(drone_autopilot ${catkin_LIBRARIES})

## GUI build

# *.ui
set(GUI_UI_FILES        
	src/UINode/tum_ardrone_gui.ui 
)

# *.qrc
set(GUI_RESOURCE_FILES        
)

# do QT stuff
ADD_DEFINITIONS( -Wall )
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
QT4_ADD_RESOURCES(GUI_RESOURCE_FILES_CPP ${GUI_RESOURCE_FILES})
QT4_WRAP_UI(GUI_UI_FILES_HPP ${GUI_UI_FILES})
QT4_WRAP_CPP(GUI_HEADER_FILES_HPP ${GUI_HEADER_FILES})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# build
add_executable(drone_gui ${GUI_SOURCE_FILES} ${GUI_RESOURCE_FILES_CPP} ${GUI_UI_FILES_HPP} ${GUI_HEADER_FILES_HPP})
add_dependencies(drone_gui GUIParams drone_stateestimation)
target_link_libraries(drone_gui ${catkin_LIBRARIES} ${QT_LIBRARIES} cvd)

#install Rules
install(TARGETS drone_autopilot drone_gui drone_stateestimation
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

