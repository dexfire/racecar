#!/usr/bin/env python

import os
import ycm_core

flags = [
'-Wall',
'-Wextra',
'-Werror',
'-fexceptions',
'-DNDEBUG',
'-std=c++11',
'-x',
'c++',
'-isystem',
'/usr/include',
'-isystem',
'/usr/local/include',
'-isystem',
'/opt/ros/' + os.getenv('ROS_DISTRO') + '/include',
'-isystem',
'/home/dexfire/racecar/devel/include',
'-isystem',
'/home/dexfire/racecar/src/6-27-2.8m.tar.gz/include',
'-isystem',
'/home/dexfire/racecar/src/7-12-2.3.tar.gz/include',
'-isystem',
'/home/dexfire/racecar/src/7-12.tar.gz/include',
'-isystem',
'/home/dexfire/racecar/src/7-13.tar.gz/include',
'-isystem',
'/home/dexfire/racecar/src/7-17-22s.tar.gz/include',
'-isystem',
'/home/dexfire/racecar/src/7-18.tar.gz/include',
'-isystem',
'/home/dexfire/racecar/src/7-20.tar.gz/include',
'-isystem',
'/home/dexfire/racecar/src/7-21-22s.tar.gz/include',
'-isystem',
'/home/dexfire/racecar/src/7-21-8:53.tar.gz/include',
'-isystem',
'/home/dexfire/racecar/src/7-4-2.97m.tar.gz/include',
'-isystem',
'/home/dexfire/racecar/src/7-4.tar.gz/include',
'-isystem',
'/home/dexfire/racecar/src/CMakeLists.txt/include',
'-isystem',
'/home/dexfire/racecar/src/ackermann_msgs/include',
'-isystem',
'/home/dexfire/racecar/src/art_imu/include',
'-isystem',
'/home/dexfire/racecar/src/art_racecar/include',
'-isystem',
'/home/dexfire/racecar/src/cam/include',
'-isystem',
'/home/dexfire/racecar/src/cam_stereo/include',
'-isystem',
'/home/dexfire/racecar/src/create_new_layers/include',
'-isystem',
'/home/dexfire/racecar/src/depthimage_to_laserscan/include',
'-isystem',
'/home/dexfire/racecar/src/k60_driver/include',
'-isystem',
'/home/dexfire/racecar/src/laser.cpp/include',
'-isystem',
'/home/dexfire/racecar/src/laser_correction/include',
'-isystem',
'/home/dexfire/racecar/src/laser_judge_obstacle/include',
'-isystem',
'/home/dexfire/racecar/src/laser_judge_obstacle 7-18.tar.gz/include',
'-isystem',
'/home/dexfire/racecar/src/laser_undistortion/include',
'-isystem',
'/home/dexfire/racecar/src/laser_undistortion.tar.gz/include',
'-isystem',
'/home/dexfire/racecar/src/ls01g/include',
'-isystem',
'/home/dexfire/racecar/src/rf2o_laser_odometry/include',
'-isystem',
'/home/dexfire/racecar/src/robot_localization/include',
'-isystem',
'/home/dexfire/racecar/src/simple_navigation_goals/include',
'-isystem',
'/home/dexfire/racecar/src/simple_navigation_goals.tar.gz/include',
'-isystem',
'/home/dexfire/racecar/src/wheel_encoder_odometry/include'
]

compilation_database_folder = ''

if os.path.exists( compilation_database_folder ):
  database = ycm_core.CompilationDatabase( compilation_database_folder )
else:
  database = None

SOURCE_EXTENSIONS = [ '.cpp', '.cxx', '.cc', '.c' ]

def DirectoryOfThisScript():
  return os.path.dirname( os.path.abspath( __file__ ) )


def MakeRelativePathsInFlagsAbsolute( flags, working_directory ):
  if not working_directory:
    return list( flags )
  new_flags = []
  make_next_absolute = False
  path_flags = [ '-isystem', '-I', '-iquote', '--sysroot=' ]
  for flag in flags:
    new_flag = flag

    if make_next_absolute:
      make_next_absolute = False
      if not flag.startswith( '/' ):
        new_flag = os.path.join( working_directory, flag )

    for path_flag in path_flags:
      if flag == path_flag:
        make_next_absolute = True
        break

      if flag.startswith( path_flag ):
        path = flag[ len( path_flag ): ]
        new_flag = path_flag + os.path.join( working_directory, path )
        break

    if new_flag:
      new_flags.append( new_flag )
  return new_flags


def IsHeaderFile( filename ):
  extension = os.path.splitext( filename )[ 1 ]
  return extension in [ '.h', '.hxx', '.hpp', '.hh' ]


def GetCompilationInfoForFile( filename ):
  if IsHeaderFile( filename ):
    basename = os.path.splitext( filename )[ 0 ]
    for extension in SOURCE_EXTENSIONS:
      replacement_file = basename + extension
      if os.path.exists( replacement_file ):
        compilation_info = database.GetCompilationInfoForFile(
          replacement_file )
        if compilation_info.compiler_flags_:
          return compilation_info
    return None
  return database.GetCompilationInfoForFile( filename )


def FlagsForFile( filename, **kwargs ):
  if database:
    compilation_info = GetCompilationInfoForFile( filename )
    if not compilation_info:
      return None

    final_flags = MakeRelativePathsInFlagsAbsolute(
      compilation_info.compiler_flags_,
      compilation_info.compiler_working_dir_ )
  else:
    relative_to = DirectoryOfThisScript()
    final_flags = MakeRelativePathsInFlagsAbsolute( flags, relative_to )

  return {
    'flags': final_flags,
    'do_cache': True
  }
