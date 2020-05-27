cmake_minimum_required(VERSION 2.8.3)
project(lg_msg_defs)

find_package(catkin REQUIRED COMPONENTS
   message_generation
   std_msgs
   geometry_msgs
)

add_message_files(
  FILES
  AdhocBrowser.msg
  AdhocBrowsers.msg
  AdhocMedia.msg
  AdhocMedias.msg
  ApplicationState.msg
  BrowserCmdArg.msg
  BrowserExtension.msg
  BrowserURL.msg
  Command.msg
  EvdevEvent.msg
  EvdevEvents.msg
  Event.msg
  GetScreenshot.msg
  ImageView.msg
  ImageViews.msg
  IrSourceInfo.msg
  LockState.msg
  MediaOverlay.msg
  MediaOverlays.msg
  PortalPose.msg
  Ready.msg
  ScreenCoordinate.msg
  Screenshot.msg
  Session.msg
  State.msg
  StringArray.msg
  TimedSwitch.msg
  WindowGeometry.msg
)

add_service_files(
  FILES
  Active.srv
  ActivityStates.srv
  BrowserPool.srv
  DesiredState.srv
  EvdevDeviceInfo.srv
  InitialUSCS.srv
  IsLocked.srv
  JSONConfigMore.srv
  JSONConfig.srv
  KmlState.srv
  Lock.srv
  MediaAppsInfo.srv
  NodeReady.srv
  Offline.srv
  PanoIdState.srv
  PlanetQuery.srv
  PlaytourQuery.srv
  TouchRoutes.srv
  UnLock.srv
  USCSMessage.srv
  ViewsyncState.srv
  Volume.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
  std_msgs
  geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
