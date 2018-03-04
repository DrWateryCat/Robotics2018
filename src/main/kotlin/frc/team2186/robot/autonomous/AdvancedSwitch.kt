package frc.team2186.robot.autonomous

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

class AdvancedSwitch : SequentialAutonomousMode("Advanced Switch", false) {
    override val actions = actionRunner {
        when (Robot.StartingPosition) {
            RobotPosition.LEFT -> {
                if (Robot.StartingSwitch === SwitchState.RIGHT) {
                    /*action(heading90())
                    action(cross(false))
                    action(heading270())
                    action(baseline(false))
                    action(dropBox())*/
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
                        Drive.leftPosition >= 137.5
                    }
                    action {
                        Drive.setForwardVelocity(0.0)
                        Drive.gyroSetpoint = Rotation2D.fromDegrees(270.0)
                        if (withinRange(Drive.gyroAngle.degrees, 265.0, 275.0)) {
                            Drive.reset()
                            true
                        }
                        false
                    }
                    action{
                        Drive.setForwardVelocity(Config.Auto.speed)
                        Drive.leftPosition >= 137.5
                    }
                }
            }
            RobotPosition.MIDDLE -> {
                if (Robot.StartingSwitch === SwitchState.LEFT) {
                    /*
                    action(heading270())
                    action(cross(true))
                    action(heading90())
                    action(baseline(false))
                    action(dropBox())*/

                    action {
                        Drive.setForwardVelocity(0.0)
                        Drive.gyroSetpoint = Rotation2D.fromDegrees(270.0)
                        if (withinRange(Drive.gyroAngle.degrees, 265.0, 275.0)) {
                            Drive.reset()
                            true
                        }
                        false
                    }
                    action{
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
                        Drive.leftPosition >= 137.5
                    }
                } else {
                    /*
                    action(heading90())
                    action(cross(true))
                    action(heading270())
                    action(baseline(false))
                    action(dropBox())*/

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
                        Drive.leftPosition >= 137.5 / 2
                    }
                    action {
                        Drive.setForwardVelocity(0.0)
                        Drive.gyroSetpoint = Rotation2D.fromDegrees(270.0)
                        if (withinRange(Drive.gyroAngle.degrees, 265.0, 275.0)) {
                            Drive.reset()
                            true
                        }
                        false
                    }
                    action{
                        Drive.setForwardVelocity(Config.Auto.speed)
                        Drive.leftPosition >= 137.5
                    }
                }
            }
            RobotPosition.RIGHT -> {
                if (Robot.StartingSwitch === SwitchState.LEFT) {
                    /*
                    action(heading270())
                    action(cross(false))
                    action(heading90())
                    action(baseline(false))
                    action(dropBox())*/

                    action {
                        Drive.setForwardVelocity(0.0)
                        Drive.gyroSetpoint = Rotation2D.fromDegrees(270.0)
                        if (withinRange(Drive.gyroAngle.degrees, 265.0, 275.0)) {
                            Drive.reset()
                            true
                        }
                        false
                    }
                    action{
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
                        Drive.leftPosition >= 137.5
                    }
                }
            }
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
