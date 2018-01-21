package frc.team2186.robot.autonomous

import frc.team2186.robot.Robot
import frc.team2186.robot.common.RobotPosition
import frc.team2186.robot.lib.common.ActionRunner
import frc.team2186.robot.lib.common.actionRunner
import frc.team2186.robot.lib.interfaces.AutonomousMode
import frc.team2186.robot.lib.interfaces.SequentialAutonomousMode
import frc.team2186.robot.lib.pathfinding.path
import frc.team2186.robot.subsystems.Drive
import frc.team2186.robot.subsystems.Grabber
import frc.team2186.robot.subsystems.Lifter

class Switch : SequentialAutonomousMode("Switch") {
    val p = path {
        waypoint {
            position = translation(0.0, 0.0)
            speed = 0.0
            marker = "Start"
        }
        waypoint {
            position = translation(60.0, -12.0)
            speed = 40.0
            marker = "Middle"
        }
        waypoint {
            position = translation(80.0, -6.0)
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
            Drive.finishedPath && Lifter.done
        }
    }
}