package frc.team2186.robot.autonomous

import edu.wpi.first.wpilibj.Timer
import frc.team2186.robot.Config
import frc.team2186.robot.Robot
import frc.team2186.robot.common.RobotPosition
import frc.team2186.robot.common.SwitchState
import frc.team2186.robot.lib.common.actionRunner
import frc.team2186.robot.lib.common.withinRange
import frc.team2186.robot.lib.interfaces.SequentialAutonomousMode
import frc.team2186.robot.lib.math.Rotation2D
import frc.team2186.robot.subsystems.Drive
import frc.team2186.robot.subsystems.Platform

class Switch : SequentialAutonomousMode("Switch", false) {
    override val actions = actionRunner {
        init {
            Drive.useGyro = true
        }
        when (Robot.StartingPosition) {
            RobotPosition.LEFT -> {
                if (Robot.StartingSwitch === SwitchState.LEFT) {
                    action {
                        Drive.setForwardVelocity(Config.Auto.speed)
                        Drive.leftPosition >= 130.0
                    }
                    action{
                        Drive.stop()
                        Platform.setpoint = -0.5
                        deltaTime >= 0.5
                    }
                    action {
                        Platform.setpoint = 0.0
                        true
                    }
                } else {
                    /*
                    action {
                        Drive.setForwardVelocity(Config.Auto.speed)
                        Drive.leftPosition >= 80.0
                    }
                    action {
                        Drive.setForwardVelocity(0.0)
                        Drive.gyroSetpoint = Rotation2D.fromDegrees(90.0)
                        if (withinRange(Drive.gyroAngle.degrees, 85.0, 95.0)) {
                            Drive.reset()
                            true
                        }
                        false
                    }
                    action {
                        Drive.setForwardVelocity(Config.Auto.speed)
                        Drive.leftPosition >= 137.5
                    }
                    action {
                        Drive.setForwardVelocity(0.0)
                        Drive.gyroSetpoint = Rotation2D.fromDegrees(270.0)
                        if (withinRange(Drive.gyroAngle.degrees, 85.0, 95.0)) {
                            Drive.reset()
                            true
                        }
                        false
                    }
                    action{
                        Drive.setForwardVelocity(Config.Auto.speed)
                        Drive.leftPosition >= 59.0
                    }*/
                    action {
                        Drive.setForwardVelocity(Config.Auto.speed)
                        Drive.leftPosition >= 130.0
                    }
                    action {
                        Drive.stop()
                        true
                    }
                }
            }
            RobotPosition.MIDDLE -> {
                if (Robot.StartingSwitch === SwitchState.LEFT) {
                    action {
                        Drive.setForwardVelocity(Config.Auto.speed)
                        Drive.leftPosition >= 80.0
                    }
                    action {
                        Drive.setForwardVelocity(0.0)
                        Drive.gyroSetpoint = Rotation2D.fromDegrees(270.0)
                        if (withinRange(Drive.gyroAngle.degrees, 85.0, 95.0)) {
                            Drive.reset()
                            true
                        }
                        false
                    }
                    action {
                        Drive.setForwardVelocity(Config.Auto.speed)
                        Drive.leftPosition >= 137.5 / 2
                    }
                    action {
                        Drive.setForwardVelocity(0.0)
                        Drive.gyroSetpoint = Rotation2D.fromDegrees(90.0)
                        if (withinRange(Drive.gyroAngle.degrees, 85.0, 95.0)) {
                            Drive.reset()
                            true
                        }
                        false
                    }
                    action{
                        Drive.setForwardVelocity(Config.Auto.speed)
                        Drive.leftPosition >= 59.0
                    }
                    action{
                        Drive.stop()
                        Platform.setpoint = -0.5
                        deltaTime >= 0.5
                    }
                    action {
                        Platform.setpoint = 0.0
                        true
                    }
                } else {
                    action {
                        Drive.setForwardVelocity(Config.Auto.speed)
                        Drive.leftPosition >= 80.0
                    }
                    action {
                        Drive.setForwardVelocity(0.0)
                        Drive.gyroSetpoint = Rotation2D.fromDegrees(90.0)
                        if (withinRange(Drive.gyroAngle.degrees, 85.0, 95.0)) {
                            Drive.reset()
                            true
                        }
                        false
                    }
                    action {
                        Drive.setForwardVelocity(Config.Auto.speed)
                        Drive.leftPosition >= 137.5 / 2
                    }
                    action{
                        Drive.setForwardVelocity(Config.Auto.speed)
                        Drive.leftPosition >= 59.0
                    }
                    action{
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
            RobotPosition.RIGHT -> {
                if (Robot.StartingSwitch === SwitchState.LEFT) {
                    /*
                    action {
                        Drive.setForwardVelocity(Config.Auto.speed)
                        Drive.leftPosition >= 80.0
                    }
                    action {
                        Drive.setForwardVelocity(0.0)
                        Drive.gyroSetpoint = Rotation2D.fromDegrees(270.0)
                        if (withinRange(Drive.gyroAngle.degrees, 85.0, 95.0)) {
                            Drive.reset()
                            true
                        }
                        false
                    }
                    action {
                        Drive.setForwardVelocity(Config.Auto.speed)
                        Drive.leftPosition >= 137.5
                    }
                    action {
                        Drive.setForwardVelocity(0.0)
                        Drive.gyroSetpoint = Rotation2D.fromDegrees(90.0)
                        if (withinRange(Drive.gyroAngle.degrees, 85.0, 95.0)) {
                            Drive.reset()
                            true
                        }
                        false
                    }
                    action{
                        Drive.setForwardVelocity(Config.Auto.speed)
                        Drive.leftPosition >= 59.0
                    }*/
                    action {
                        Drive.setForwardVelocity(Config.Auto.speed)
                        Drive.leftPosition >= 130.5
                    }
                    action {
                        Drive.stop()
                        true
                    }
                } else {
                    action {
                        Drive.setForwardVelocity(Config.Auto.speed)
                        Drive.leftPosition >= 130.0
                    }
                    action{
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
        }
    }
}
