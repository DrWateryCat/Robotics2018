package frc.team2186.robot.autonomous

import frc.team2186.robot.Config
import frc.team2186.robot.Robot
import frc.team2186.robot.common.RobotPosition
import frc.team2186.robot.common.SwitchState
import frc.team2186.robot.lib.common.actionRunner
import frc.team2186.robot.lib.interfaces.SequentialAutonomousMode
import frc.team2186.robot.lib.math.Translation2D
import frc.team2186.robot.lib.pathfinding.Path
import frc.team2186.robot.lib.pathfinding.PathBuilder
import frc.team2186.robot.subsystems.Drive
import frc.team2186.robot.subsystems.Platform

class PurePursuitAuto : SequentialAutonomousMode("Pure Pursuit", false) {
    /*
    val p = path {
        waypoint {
            speed = 80.0
            when (Robot.StartingPosition) {
                RobotPosition.LEFT -> {
                    position = if (Robot.StartingSwitch == SwitchState.LEFT) {
                        translation(120.0, 3.0)
                    } else {
                        translation(120.0, 137.5)
                    }
                }
                RobotPosition.MIDDLE -> {
                    position = if(Robot.StartingSwitch == SwitchState.LEFT) {
                        translation(120.0, -137.5 / 2)
                    } else {
                        translation(120.0, 137.5 / 2)
                    }
                }
                RobotPosition.RIGHT -> {
                    position = if(Robot.StartingSwitch == SwitchState.LEFT) {
                        translation(120.0, -137.5)
                    } else {
                        translation(120.0, -3.0)
                    }
                }
            }
        }
    }*/
    val p = PathBuilder().apply {
        val (x, y) = when(Robot.StartingPosition) {
            RobotPosition.LEFT -> {
                if (Robot.StartingSwitch == SwitchState.RIGHT) {
                    Pair(120.0, 137.0)
                } else {
                    Pair(120.0, 3.0)
                }
            }
            RobotPosition.MIDDLE -> {
                if (Robot.StartingSwitch == SwitchState.RIGHT) {
                    Pair(120.0, 137.5 / 2)
                } else {
                    Pair(120.0, -137.5 / 2)
                }
            }
            RobotPosition.RIGHT -> {
                if (Robot.StartingSwitch == SwitchState.RIGHT) {
                    Pair(120.0, -3.0)
                } else {
                    Pair(120.0, -137.0)
                }
            }
        }
        addWaypoint(Path.Waypoint(Translation2D(x, y), Config.Auto.speed))
    }.build()
    override val actions = actionRunner {
        init {
            Drive.followPath(p)
        }
        action {
            Drive.finishedPath
        }
        action {
            Drive.tankDrive(0.0, 0.0)

            Platform.setpoint = -0.5
            Platform.isBarUp
        }
        action {
            Platform.setpoint = 0.0
            true
        }
    }
}