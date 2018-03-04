package frc.team2186.robot.autonomous

import frc.team2186.robot.Robot
import frc.team2186.robot.common.RobotPosition
import frc.team2186.robot.common.SwitchState
import frc.team2186.robot.lib.common.ActionRunner
import frc.team2186.robot.lib.common.actionRunner
import frc.team2186.robot.lib.interfaces.SequentialAutonomousMode
import frc.team2186.robot.lib.pathfinding.path
import frc.team2186.robot.subsystems.Drive
import frc.team2186.robot.subsystems.Platform

class PurePursuitAuto : SequentialAutonomousMode("Pure Pursuit", false) {
    val p = path {
        waypoint {
            speed = 55.0
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
    }
    override val actions = actionRunner {
        init {
            Drive.followPath(p)
        }
        action {
            Drive.pathFinished
        }
        action {
            Drive.stop()

            Platform.setpoint = -0.5
            deltaTime >= 0.5
        }
        action {
            Platform.setpoint = 0.0
            true
        }
    }
}