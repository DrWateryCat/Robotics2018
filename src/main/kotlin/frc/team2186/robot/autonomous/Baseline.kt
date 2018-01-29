package frc.team2186.robot.autonomous

import frc.team2186.robot.Robot
import frc.team2186.robot.common.RobotPosition
import frc.team2186.robot.common.Waypoints
import frc.team2186.robot.lib.common.ActionRunner
import frc.team2186.robot.lib.common.IterativeAutoAction
import frc.team2186.robot.lib.common.SequentialActionRunner
import frc.team2186.robot.lib.common.actionRunner
import frc.team2186.robot.lib.interfaces.AutonomousMode
import frc.team2186.robot.lib.interfaces.SequentialAutonomousMode
import frc.team2186.robot.lib.math.Translation2D
import frc.team2186.robot.lib.pathfinding.Path
import frc.team2186.robot.lib.pathfinding.Waypoint
import frc.team2186.robot.lib.pathfinding.path
import frc.team2186.robot.subsystems.Drive

class Baseline : SequentialAutonomousMode("Baseline") {
    val p = path {
        waypoint {
            position = translation(0.0, 0.0)
            speed = 0.0
            marker = "Start"
        }
        waypoint {
            position = Waypoints.baseline.scale(0.5)
            speed = 50.0
            marker = "Middle"
        }
        waypoint {
            position = Waypoints.baseline
            speed = 0.0
            marker = "End"
        }
    }
    override val actions = actionRunner {
        init {
            Drive.reset()
            Drive.followPath(p)
        }
        action {
            Drive.finishedPath
        }
        action {
            Drive.stop()

            Drive.stopped
        }
    }
}