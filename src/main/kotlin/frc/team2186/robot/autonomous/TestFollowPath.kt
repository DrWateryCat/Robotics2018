package frc.team2186.robot.autonomous

import frc.team2186.robot.lib.interfaces.AutonomousMode
import frc.team2186.robot.lib.math.Translation2D
import frc.team2186.robot.lib.pathfinding.Path
import frc.team2186.robot.lib.pathfinding.Waypoint
import frc.team2186.robot.lib.pathfinding.path
import frc.team2186.robot.subsystems.Drive

class TestFollowPath : AutonomousMode("Path Follower Test") {
    override fun init() {
        Drive.followPath(path {
            waypoint {
                position = translation(0.0, 0.0)
                speed = 0.0
                marker = "Start"
            }
        })
    }

    override fun update() {
        if (done()) {
            Drive.stop()
        }
    }

    override fun done(): Boolean = Drive.finishedPath
}