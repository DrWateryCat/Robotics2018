package frc.team2186.robot.autonomous

import frc.team2186.robot.Config
import frc.team2186.robot.Robot
import frc.team2186.robot.common.ScaleState
import frc.team2186.robot.common.SwitchState
import frc.team2186.robot.common.Waypoints
import frc.team2186.robot.lib.common.actionRunner
import frc.team2186.robot.lib.interfaces.SequentialAutonomousMode
import frc.team2186.robot.lib.math.Rotation2D
import frc.team2186.robot.lib.pathfinding.path
import frc.team2186.robot.subsystems.Drive
import frc.team2186.robot.subsystems.Lifter

class Scale : SequentialAutonomousMode("Scale") {
    val p = path {
        val leftOrRight = if (Robot.StartingScale == ScaleState.LEFT) SwitchState.LEFT else SwitchState.RIGHT
        val switch = Waypoints.switch(Robot.StartingPosition, leftOrRight)
        startingPoint()
        waypoint {
            position = switch.scale(0.5)
            speed = 60.0
            marker = "Halfway to switch"
        }
        waypoint {
            position = switch.translateBy(translation(0.0, 12.0 * if (leftOrRight == SwitchState.LEFT) -1 else 1))
            speed = 60.0
            marker = "Switch"
        }
        waypoint {
            position = Waypoints.scale(Robot.StartingPosition, Robot.StartingScale)
            speed = 0.0
            marker = "Scale"
        }
    }

    override val actions = actionRunner {
        init {
            Drive.followPath(p)
        }
        action {
            Drive.finishedPath
        }
        action {
            Drive.stop()

            Drive.stopped
        }
        action {
            Drive.gyroSetpoint = Rotation2D.fromDegrees(90.0)

            Drive.gyroAngle.degrees == 90.0
        }
        action {
            Lifter.set(1.0)

            Lifter.done
        }
        action {
            Lifter.barUp()

            Lifter.isBarUp
        }
        action {
            Lifter.barDown()

            Lifter.isBarDown
        }
        action {
            Lifter.set(0.0)

            Lifter.done
        }
    }
}