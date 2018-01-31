package frc.team2186.robot.autonomous

import frc.team2186.robot.common.Waypoints
import frc.team2186.robot.lib.common.actionRunner
import frc.team2186.robot.lib.interfaces.SequentialAutonomousMode
import frc.team2186.robot.lib.pathfinding.path
import frc.team2186.robot.subsystems.Drive

class Baseline : SequentialAutonomousMode("Baseline") {
    val p = path {
        pathTo(Waypoints.baseline, 50.0)
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