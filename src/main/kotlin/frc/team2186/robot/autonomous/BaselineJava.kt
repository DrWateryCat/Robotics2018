package frc.team2186.robot.autonomous

import frc.team2186.robot.Robot
import frc.team2186.robot.common.RobotPosition
import frc.team2186.robot.common.SwitchState
import frc.team2186.robot.lib.common.actionRunner
import frc.team2186.robot.lib.interfaces.AutonomousMode
import frc.team2186.robot.lib.math.Rotation2D
import frc.team2186.robot.subsystems.Drive
import frc.team2186.robot.subsystems.Platform

class BaselineJava : AutonomousMode("Baseline in Java", false) {
    val actions = actionRunner {
        action {
            Drive.setForwardVelocity(20.0)
            Drive.leftPosition >= 80.0
        }
        if (((Robot.StartingSwitch == SwitchState.LEFT) and (Robot.StartingPosition == RobotPosition.LEFT)) or ((Robot.StartingSwitch == SwitchState.RIGHT) and (Robot.StartingPosition == RobotPosition.RIGHT))) {
            action {
                Platform.setpoint = -0.5
                deltaTime >= 1.0
            }
            action {
                Platform.setpoint = 0.0
                true
            }
        }
        action {
            Drive.stop()
            true
        }
    }

    override fun init() {
        Drive.useGyro = true
        Drive.positionPid = false
        actions.init()
        println(actions)
    }

    override fun update() {
        actions.update()
    }

    override fun done() = actions.done
}
