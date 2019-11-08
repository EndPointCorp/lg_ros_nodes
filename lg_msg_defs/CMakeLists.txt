cmake_minimum_required(VERSION 2.8.3)
project(lg_msg_defs)

find_package(catkin REQUIRED COMPONENTS
   message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  AdhocBrowser.msg
  AdhocBrowsers.msg
  AdhocMedia.msg
  AdhocMedias.msg
  ApplicationState.msg
  BrowserCmdArg.msg
  BrowserExtension.msg
  Command.msg
  EvdevEvent.msg
  EvdevEvents.msg
  Event.msg
  GetScreenshot.msg
  ImageView.msg
  ImageViews.msg
  IrSourceInfo.msg
  LockState.msg
  PortalPose.msg
  Ready.msg
  Screenshot.msg
  Session.msg
  State.msg
  StringArray.msg
  TimedSwitch.msg
  WindowGeometry.msg
)

generate_messages()


