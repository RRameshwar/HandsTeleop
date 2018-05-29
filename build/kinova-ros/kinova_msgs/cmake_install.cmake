# Install script for directory: /home/srl-jacoarm/Desktop/SRL-JacoArm/src/kinova-ros/kinova_msgs

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/home/srl-jacoarm/Desktop/SRL-JacoArm/install")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kinova_msgs/msg" TYPE FILE FILES
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/src/kinova-ros/kinova_msgs/msg/FingerPosition.msg"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/src/kinova-ros/kinova_msgs/msg/JointAngles.msg"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/src/kinova-ros/kinova_msgs/msg/JointVelocity.msg"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/src/kinova-ros/kinova_msgs/msg/JointTorque.msg"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/src/kinova-ros/kinova_msgs/msg/KinovaPose.msg"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/src/kinova-ros/kinova_msgs/msg/PoseVelocity.msg"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/src/kinova-ros/kinova_msgs/msg/CartesianForce.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kinova_msgs/srv" TYPE FILE FILES
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/src/kinova-ros/kinova_msgs/srv/Start.srv"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/src/kinova-ros/kinova_msgs/srv/Stop.srv"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/src/kinova-ros/kinova_msgs/srv/HomeArm.srv"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/src/kinova-ros/kinova_msgs/srv/SetForceControlParams.srv"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/src/kinova-ros/kinova_msgs/srv/SetEndEffectorOffset.srv"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/src/kinova-ros/kinova_msgs/srv/SetNullSpaceModeState.srv"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/src/kinova-ros/kinova_msgs/srv/SetTorqueControlMode.srv"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/src/kinova-ros/kinova_msgs/srv/SetTorqueControlParameters.srv"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/src/kinova-ros/kinova_msgs/srv/ClearTrajectories.srv"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/src/kinova-ros/kinova_msgs/srv/ZeroTorques.srv"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/src/kinova-ros/kinova_msgs/srv/AddPoseToCartesianTrajectory.srv"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/src/kinova-ros/kinova_msgs/srv/RunCOMParametersEstimation.srv"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kinova_msgs/action" TYPE FILE FILES
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/src/kinova-ros/kinova_msgs/action/ArmJointAngles.action"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/src/kinova-ros/kinova_msgs/action/ArmPose.action"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/src/kinova-ros/kinova_msgs/action/Arm_KinovaPose.action"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/src/kinova-ros/kinova_msgs/action/SetFingersPosition.action"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kinova_msgs/msg" TYPE FILE FILES
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/devel/share/kinova_msgs/msg/ArmJointAnglesAction.msg"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/devel/share/kinova_msgs/msg/ArmJointAnglesActionGoal.msg"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/devel/share/kinova_msgs/msg/ArmJointAnglesActionResult.msg"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/devel/share/kinova_msgs/msg/ArmJointAnglesActionFeedback.msg"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/devel/share/kinova_msgs/msg/ArmJointAnglesGoal.msg"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/devel/share/kinova_msgs/msg/ArmJointAnglesResult.msg"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/devel/share/kinova_msgs/msg/ArmJointAnglesFeedback.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kinova_msgs/msg" TYPE FILE FILES
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/devel/share/kinova_msgs/msg/ArmPoseAction.msg"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/devel/share/kinova_msgs/msg/ArmPoseActionGoal.msg"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/devel/share/kinova_msgs/msg/ArmPoseActionResult.msg"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/devel/share/kinova_msgs/msg/ArmPoseActionFeedback.msg"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/devel/share/kinova_msgs/msg/ArmPoseGoal.msg"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/devel/share/kinova_msgs/msg/ArmPoseResult.msg"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/devel/share/kinova_msgs/msg/ArmPoseFeedback.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kinova_msgs/msg" TYPE FILE FILES
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/devel/share/kinova_msgs/msg/Arm_KinovaPoseAction.msg"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/devel/share/kinova_msgs/msg/Arm_KinovaPoseActionGoal.msg"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/devel/share/kinova_msgs/msg/Arm_KinovaPoseActionResult.msg"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/devel/share/kinova_msgs/msg/Arm_KinovaPoseActionFeedback.msg"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/devel/share/kinova_msgs/msg/Arm_KinovaPoseGoal.msg"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/devel/share/kinova_msgs/msg/Arm_KinovaPoseResult.msg"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/devel/share/kinova_msgs/msg/Arm_KinovaPoseFeedback.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kinova_msgs/msg" TYPE FILE FILES
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/devel/share/kinova_msgs/msg/SetFingersPositionAction.msg"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/devel/share/kinova_msgs/msg/SetFingersPositionActionGoal.msg"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/devel/share/kinova_msgs/msg/SetFingersPositionActionResult.msg"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/devel/share/kinova_msgs/msg/SetFingersPositionActionFeedback.msg"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/devel/share/kinova_msgs/msg/SetFingersPositionGoal.msg"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/devel/share/kinova_msgs/msg/SetFingersPositionResult.msg"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/devel/share/kinova_msgs/msg/SetFingersPositionFeedback.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kinova_msgs/cmake" TYPE FILE FILES "/home/srl-jacoarm/Desktop/SRL-JacoArm/build/kinova-ros/kinova_msgs/catkin_generated/installspace/kinova_msgs-msg-paths.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/srl-jacoarm/Desktop/SRL-JacoArm/devel/include/kinova_msgs")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/srl-jacoarm/Desktop/SRL-JacoArm/devel/share/common-lisp/ros/kinova_msgs")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/srl-jacoarm/Desktop/SRL-JacoArm/devel/lib/python2.7/dist-packages/kinova_msgs")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/srl-jacoarm/Desktop/SRL-JacoArm/devel/lib/python2.7/dist-packages/kinova_msgs")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/srl-jacoarm/Desktop/SRL-JacoArm/build/kinova-ros/kinova_msgs/catkin_generated/installspace/kinova_msgs.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kinova_msgs/cmake" TYPE FILE FILES "/home/srl-jacoarm/Desktop/SRL-JacoArm/build/kinova-ros/kinova_msgs/catkin_generated/installspace/kinova_msgs-msg-extras.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kinova_msgs/cmake" TYPE FILE FILES
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/build/kinova-ros/kinova_msgs/catkin_generated/installspace/kinova_msgsConfig.cmake"
    "/home/srl-jacoarm/Desktop/SRL-JacoArm/build/kinova-ros/kinova_msgs/catkin_generated/installspace/kinova_msgsConfig-version.cmake"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kinova_msgs" TYPE FILE FILES "/home/srl-jacoarm/Desktop/SRL-JacoArm/src/kinova-ros/kinova_msgs/package.xml")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

