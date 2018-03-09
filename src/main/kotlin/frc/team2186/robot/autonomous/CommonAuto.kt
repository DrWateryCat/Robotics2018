package frc.team2186.robot.autonomous

import frc.team2186.robot.Config
import frc.team2186.robot.lib.common.withinRange
import frc.team2186.robot.subsystems.Drive
import frc.team2186.robot.subsystems.Platform

/*
object CommonAuto {
    fun goDistance(distance: Double): IterativeAutoAction {
        return IterativeAutoAction {
            OldDrive.setForwardVelocity(Config.Auto.speed)
            OldDrive.leftPosition > distance
        }
    }

    fun turnToAngle(angle: Double): IterativeAutoAction {
        return IterativeAutoAction {
            OldDrive.gyroSetpoint = Rotation2D.fromDegrees(angle)

            OldDrive.gyroAngle.degrees == angle
        }
    }

    fun stop(): Function0<Unit> {
        return {
            OldDrive.stop()
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
*/

fun distance(length: Double): () -> Boolean {
    return {
        Drive.setForwardVelocity(Config.Auto.speed)
        Drive.leftPosition >= length
    }
}

fun turnToAngle(angle: Double): () -> Boolean {
    return {
        Drive.setVelocityVector(0.0, angle)
        withinRange(Drive.heading, angle - 1, angle + 1)
    }
}

fun baseline(half: Boolean) = distance(85.0 / (if (half) 2 else 1))
fun cross(half: Boolean) = distance(137.5 / (if (half) 2 else 1))
fun heading90() = turnToAngle(90.0)
fun heading270() = turnToAngle(270.0)
fun moveOne() = distance(40.0)
fun moveTwo() = distance(45.0)
fun dropBox() = {
    Platform.setpoint = -0.5
    if (Platform.isBarUp) {
        Platform.setpoint = 0.0
        true
    }
    false
}