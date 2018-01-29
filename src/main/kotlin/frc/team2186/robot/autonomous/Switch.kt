package frc.team2186.robot.autonomous

import frc.team2186.robot.Robot
import frc.team2186.robot.common.Waypoints
import frc.team2186.robot.lib.common.actionRunner
import frc.team2186.robot.lib.interfaces.SequentialAutonomousMode
import frc.team2186.robot.lib.pathfinding.path
import frc.team2186.robot.subsystems.Drive
import frc.team2186.robot.subsystems.Lifter

class Switch : SequentialAutonomousMode("Switch") {
    val p = path {
        waypoint {
            position = translation(0.0, 0.0)
            speed = 0.0
            marker = "Start"
        }
        waypoint {
            position = Waypoints.switch(Robot.StartingPosition, Robot.StartingSwitch).scale(0.5)
            speed = 60.0
            marker = "Middle"
        }
        waypoint {
            position = Waypoints.switch(Robot.StartingPosition, Robot.StartingSwitch)
            speed = 0.0
            marker = "End"
        }
    }

    override val actions = actionRunner {
        init {
            Drive.followPath(p)
        }
        action {
            Lifter.set(0.25)

            Drive.finishedPath and Lifter.done
        }
        action {
            Drive.stop()
            Lifter.barUp()

            Lifter.isBarUp and Drive.stopped
        }
        action {
            Lifter.barDown()

            Lifter.isBarDown
        }
    }
}