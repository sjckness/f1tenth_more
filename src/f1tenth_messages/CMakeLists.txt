cmake_minimum_required(VERSION 3.5)
project(f1tenth_messages)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(f1tenth_messages
  "msg/SystemStatus.msg"
  "msg/BatteryStatus.msg"
  "msg/Obstacle2D.msg"
  "msg/Obstacle2DArray.msg"
  "msg/MissionStatus.msg"
  "msg/WallDetection.msg"
  "msg/WallArray.msg"
  "msg/TurnGoal.msg"
  "msg/DriveCommand.msg"
  "msg/BoundaryConstraint.msg"
  "msg/BoundaryConstraintArray.msg"
  "msg/MpcSolverStatus.msg"
  "msg/BehaviorTreeStatus.msg"
  "msg/WallLineFit.msg"
  "msg/WallEstimate.msg"
  "msg/WallTrack.msg"
  "msg/WallDistance.msg"
  "msg/WallSegment.msg"
  "msg/MultiWallTrack.msg"
  "msg/MultiWallTrackArray.msg"
  "msg/StereoColumns.msg"
  "msg/WallObservation.msg"
  "msg/WallObservationArray.msg"
  "msg/ObjectGoal.msg"
  "msg/ObjectApproachStatus.msg"
  "msg/DriveClamp.msg"
  "msg/ReverseStatus.msg"
  "msg/SpeedChannelStatus.msg"
  "msg/MoveOutcome.msg"
  "msg/Door.msg"
  "msg/DoorArray.msg"
  "msg/SemanticObject.msg"
  "msg/SemanticObjectArray.msg"
  "msg/MpcParamUpdate.msg"
  "srv/RestartComponent.srv"
  "srv/ComponentControl.srv"
  "srv/RunDiagnostics.srv"
  "srv/LoadMission.srv"
  "srv/RequestReverse.srv"
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
