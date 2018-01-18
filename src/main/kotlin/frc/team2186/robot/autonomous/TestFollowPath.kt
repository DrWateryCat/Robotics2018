package frc.team2186.robot.autonomous

import frc.team2186.robot.lib.interfaces.AutonomousMode
import frc.team2186.robot.lib.math.Translation2D
import frc.team2186.robot.lib.pathfinding.Path
import frc.team2186.robot.subsystems.Drive

class TestFollowPath : AutonomousMode() {
    override fun init() {
        val waypoints = ArrayList<Path.Waypoint>()
        waypoints.add(Path.Waypoint(Translation2D(0.0, 0.0), 0.0, "Start"))
        waypoints.add(Path.Waypoint(Translation2D(2.5, 60.0), 40.0, "Middle"))
        waypoints.add(Path.Waypoint(Translation2D(5.0, 120.0), 0.0, "End"))

        val path = Path(waypoints)
        Drive.followPath(path)
    }

    override fun update() {
        if (Drive.finishedPath) {
            Drive.stop()
        }
    }
}