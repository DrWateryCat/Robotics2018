package frc.team2186.robot.autonomous

import frc.team2186.robot.Config
import frc.team2186.robot.lib.common.IterativeAutoAction
import frc.team2186.robot.lib.math.Rotation2D
import frc.team2186.robot.subsystems.Drive
import frc.team2186.robot.subsystems.Platform

object CommonAuto {
    fun goDistance(distance: Double): IterativeAutoAction {
        return IterativeAutoAction {
            Drive.setForwardVelocity(Config.Auto.speed)
            Drive.leftPosition > distance
        }
    }

    fun turnToAngle(angle: Double): IterativeAutoAction {
        return IterativeAutoAction {
            Drive.gyroSetpoint = Rotation2D.fromDegrees(angle)

            Drive.gyroAngle.degrees == angle
        }
    }

    fun stop(): Function0<Unit> {
        return {
            Drive.stop()
            Unit
        }
    }

    fun baseline(half: Boolean): IterativeAutoAction {
        return if (half) goDistance((103 / 2).toDouble()) else goDistance(103.0)
    }

    fun crossBoard(half: Boolean): IterativeAutoAction {
        return if (half) goDistance(137.5 / 2) else goDistance(137.5)
    }

    fun heading90(): IterativeAutoAction {
        return turnToAngle(90.0)
    }

    fun heading270(): IterativeAutoAction {
        return turnToAngle(270.0)
    }

    fun dropBox(): IterativeAutoAction {
        return IterativeAutoAction {
            Platform.setpoint = 0.25

            if (Platform.isBarUp) {
                Platform.setpoint = -0.25
                Platform.isBarDown
            }
            false
        }
    }

    fun moveOne(): IterativeAutoAction {
        return goDistance(80.0)
    }

    fun moveTwo(): IterativeAutoAction {
        return goDistance(59.0)
    }
}
