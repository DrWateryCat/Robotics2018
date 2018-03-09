package frc.team2186.robot.autonomous

import frc.team2186.robot.Robot
import frc.team2186.robot.common.RobotPosition
import frc.team2186.robot.common.SwitchState
import frc.team2186.robot.lib.common.actionRunner
import frc.team2186.robot.lib.interfaces.SequentialAutonomousMode
import frc.team2186.robot.subsystems.Drive
import frc.team2186.robot.subsystems.Platform

class Switch : SequentialAutonomousMode("Switch", false) {
    override val actions = actionRunner {
        when (Robot.StartingPosition) {
            RobotPosition.LEFT -> {
                if (Robot.StartingSwitch === SwitchState.LEFT) {
                    action(baseline(false))
                } else {
                    /*
                    action {
                        OldDrive.setForwardVelocity(Config.Auto.speed)
                        OldDrive.leftPosition >= 80.0
                    }
                    action {
                        OldDrive.setForwardVelocity(0.0)
                        OldDrive.gyroSetpoint = Rotation2D.fromDegrees(90.0)
                        if (withinRange(OldDrive.gyroAngle.degrees, 85.0, 95.0)) {
                            OldDrive.reset()
                            true
                        }
                        false
                    }
                    action {
                        OldDrive.setForwardVelocity(Config.Auto.speed)
                        OldDrive.leftPosition >= 137.5
                    }
                    action {
                        OldDrive.setForwardVelocity(0.0)
                        OldDrive.gyroSetpoint = Rotation2D.fromDegrees(270.0)
                        if (withinRange(OldDrive.gyroAngle.degrees, 85.0, 95.0)) {
                            OldDrive.reset()
                            true
                        }
                        false
                    }
                    action{
                        OldDrive.setForwardVelocity(Config.Auto.speed)
                        OldDrive.leftPosition >= 59.0
                    }*/

                    action(moveOne())
                    action(heading90())
                    action(cross(true))
                    action(heading270())
                    action(moveTwo())
                }
            }
            RobotPosition.MIDDLE -> {
                if (Robot.StartingSwitch === SwitchState.LEFT) {
                    /*
                    action {
                        OldDrive.setForwardVelocity(Config.Auto.speed)
                        OldDrive.leftPosition >= 80.0
                    }
                    action {
                        OldDrive.setForwardVelocity(0.0)
                        OldDrive.gyroSetpoint = Rotation2D.fromDegrees(270.0)
                        if (withinRange(OldDrive.gyroAngle.degrees, 85.0, 95.0)) {
                            OldDrive.reset()
                            true
                        }
                        false
                    }
                    action {
                        OldDrive.setForwardVelocity(Config.Auto.speed)
                        OldDrive.leftPosition >= 137.5 / 2
                    }
                    action {
                        OldDrive.setForwardVelocity(0.0)
                        OldDrive.gyroSetpoint = Rotation2D.fromDegrees(90.0)
                        if (withinRange(OldDrive.gyroAngle.degrees, 85.0, 95.0)) {
                            OldDrive.reset()
                            true
                        }
                        false
                    }
                    action{
                        OldDrive.setForwardVelocity(Config.Auto.speed)
                        OldDrive.leftPosition >= 59.0
                    }
                    action{
                        OldDrive.stop()
                        Platform.setpoint = -0.5
                        deltaTime >= 0.5
                    }
                    action {
                        Platform.setpoint = 0.0
                        true
                    }*/

                    action(moveOne())
                    action(heading270())
                    action(cross(true))
                    action(heading90())
                    action(moveTwo())
                } else {
                    /*
                    action {
                        OldDrive.setForwardVelocity(Config.Auto.speed)
                        OldDrive.leftPosition >= 80.0
                    }
                    action {
                        OldDrive.setForwardVelocity(0.0)
                        OldDrive.gyroSetpoint = Rotation2D.fromDegrees(90.0)
                        if (withinRange(OldDrive.gyroAngle.degrees, 85.0, 95.0)) {
                            OldDrive.reset()
                            true
                        }
                        false
                    }
                    action {
                        OldDrive.setForwardVelocity(Config.Auto.speed)
                        OldDrive.leftPosition >= 137.5 / 2
                    }
                    action{
                        OldDrive.setForwardVelocity(Config.Auto.speed)
                        OldDrive.leftPosition >= 59.0
                    }
                    action{
                        OldDrive.stop()
                        Platform.setpoint = -0.5
                        deltaTime >= 0.5
                    }
                    action {
                        Platform.setpoint = 0.0
                        true
                    }*/

                    action(moveOne())
                    action(heading90())
                    action(cross(true))
                    action(heading270())
                    action(moveTwo())
                }
            }
            RobotPosition.RIGHT -> {
                if (Robot.StartingSwitch === SwitchState.LEFT) {
                    /*
                    action {
                        OldDrive.setForwardVelocity(Config.Auto.speed)
                        OldDrive.leftPosition >= 80.0
                    }
                    action {
                        OldDrive.setForwardVelocity(0.0)
                        OldDrive.gyroSetpoint = Rotation2D.fromDegrees(270.0)
                        if (withinRange(OldDrive.gyroAngle.degrees, 85.0, 95.0)) {
                            OldDrive.reset()
                            true
                        }
                        false
                    }
                    action {
                        OldDrive.setForwardVelocity(Config.Auto.speed)
                        OldDrive.leftPosition >= 137.5
                    }
                    action {
                        OldDrive.setForwardVelocity(0.0)
                        OldDrive.gyroSetpoint = Rotation2D.fromDegrees(90.0)
                        if (withinRange(OldDrive.gyroAngle.degrees, 85.0, 95.0)) {
                            OldDrive.reset()
                            true
                        }
                        false
                    }
                    action{
                        OldDrive.setForwardVelocity(Config.Auto.speed)
                        OldDrive.leftPosition >= 59.0
                    }*/
                    action(moveOne())
                    action(heading270())
                    action(cross(false))
                    action(heading90())
                    action(moveTwo())
                } else {
                    action(baseline(true))
                }
            }
        }
        action {
            Platform.setpoint = -0.5
            deltaTime >= 1.0
        }
        action {
            Platform.setpoint = 0.0
            true
        }
        actionComplete {
            Drive.tankDrive(0.0, 0.0)
            Drive.reset()
        }
    }
}
