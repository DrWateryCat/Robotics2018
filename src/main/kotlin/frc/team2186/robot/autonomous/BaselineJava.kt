package frc.team2186.robot.autonomous

import frc.team2186.robot.lib.common.actionRunner
import frc.team2186.robot.lib.interfaces.AutonomousMode
import frc.team2186.robot.lib.math.Rotation2D
import frc.team2186.robot.subsystems.Drive
import frc.team2186.robot.subsystems.Platform

class BaselineJava : AutonomousMode("Baseline in Java", false) {
    val actions = actionRunner {
        action {
            Drive.gyroSetpoint = Rotation2D.fromDegrees(90.0)
            Drive.leftSetpoint = 0.15
            Drive.rightSetpoint = 0.15

            Drive.leftPosition > 100.0
        }
        action {
            Drive.stop()
            Platform.setpoint = -0.5

            deltaTime > 0.5
        }
        action {
            Platform.setpoint = 0.0
            true
        }
        action {
            Drive.gyroSetpoint = Rotation2D.fromDegrees(180.0)
            Drive.leftSetpoint = -0.15
            Drive.rightSetpoint = 0.15

            Drive.onTarget
        }
        action {
            Drive.stop()
            true
        }
    }

    override fun init() {
        Drive.useGyro = true
        Drive.positionPid = true
        actions.init()
        println(actions)
    }

    override fun update() {
        actions.update()
    }

    override fun done() = actions.done
}
