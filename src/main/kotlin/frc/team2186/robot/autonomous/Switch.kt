package frc.team2186.robot.autonomous

import frc.team2186.robot.Robot
import frc.team2186.robot.common.RobotPosition
import frc.team2186.robot.lib.common.actionRunner
import frc.team2186.robot.lib.interfaces.AutonomousMode
import frc.team2186.robot.lib.pathfinding.path
import frc.team2186.robot.subsystems.Drive

class Switch : AutonomousMode() {
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

    val actions = actionRunner {
        init {
            Drive.followPath(p)
        }
        action {
            Drive.finishedPath
        }
    }
    override fun init() {
        actions.init()
    }

    override fun update() {
        actions.update()
    }

    override fun done(): Boolean {
        return true
    }
}