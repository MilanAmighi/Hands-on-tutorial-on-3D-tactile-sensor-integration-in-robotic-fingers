# Windows launcher for the full tactile-robot system.
#
# rviz2 reliably dies with exit code 3221226505 (STATUS_STACK_BUFFER_OVERRUN, abort() from
# ucrtbase.dll) within a few seconds when spawned as a child of `ros2 launch`'s
# ExecuteProcess on Windows -- reproduced with no other nodes running and regardless of
# working directory, so it's specific to how ros2 launch spawns child processes on Windows.
# rviz2 launched directly (no intermediate shell) is unaffected. So on Windows we start
# rviz2 here directly and pass rviz:=false to the launch file for everything else.
#
# Run from Codes/ros2_ws (this script's directory).

param(
    [string]$LaunchFile = "full_system.launch.py",
    [string[]]$ExtraArgs = @()
)

$env:AMENT_PREFIX_PATH = (Resolve-Path "install\ros2_tactile_robot").Path + ";" + $env:AMENT_PREFIX_PATH
$env:ROS_LOCALHOST_ONLY = "1"

$rvizConfig = "install\ros2_tactile_robot\share\ros2_tactile_robot\rviz\tactile_viz.rviz"
Start-Process -FilePath "rviz2" -ArgumentList @("-d", $rvizConfig)

ros2 launch ros2_tactile_robot $LaunchFile rviz:=false @ExtraArgs
