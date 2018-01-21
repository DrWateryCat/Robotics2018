package frc.team2186.robot.autonomous

import frc.team2186.robot.lib.interfaces.AutonomousMode
import frc.team2186.robot.lib.math.Translation2D
import frc.team2186.robot.lib.pathfinding.Path
import frc.team2186.robot.lib.pathfinding.Waypoint
import frc.team2186.robot.subsystems.Drive

class TestFollowPath : AutonomousMode() {
    override fun init() {
        val path = Path(
                Waypoint(Translation2D(0.0, 0.0), 0.0, "Start"),
                Waypoint(Translation2D(2.5 * 12, 5.0 * 12), 40.0, "Middle"),
                Waypoint(Translation2D(5.0 * 12, 10.0 * 12), 0.0, "End")
        )
        path.create()
        Drive.followPath(path)
    }

    override fun update() {
        if (done()) {
            Drive.stop()
        }
    }

    override fun done(): Boolean = Drive.finishedPath
}