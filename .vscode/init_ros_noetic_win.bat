:: Copyright: William Navaraj, 2022, CC-By-NC 3.0
@echo off
call C:\opt\ros\noetic\x64\tools\clink.1.3.2.222baa\clink inject
cls
:: PROMPT rosN$g
echo ****Setting up your ROS Environment****
:: call "C:\\Program Files (x86)\\Microsoft Visual Studio\\2019\\Enterprise\\Common7\\Tools\\VsDevCmd.bat" -arch=amd64 -host_arch=amd64
call "C:\\Program Files (x86)\\Microsoft Visual Studio\\2019\\Community\\Common7\\Tools\\VsDevCmd.bat" -arch=amd64 -host_arch=amd64
:: call "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvars64.bat"
call c:\\opt\\ros\\noetic\\x64\\setup.bat
cls
set ChocolateyInstall=c:\\opt\\chocolatey
:: C:\Python39\Lib\site-packages\PyQt5\Qt5\plugins\platforms
doskey catkin_make=catkin_make --use-nmake $*
doskey ls=dir
cls
doskey cat=type
:: roslaunch gazebo_ros empty_world.launch
