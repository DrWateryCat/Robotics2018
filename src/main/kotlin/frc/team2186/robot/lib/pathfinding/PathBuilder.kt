package frc.team2186.robot.lib.pathfinding

class PathBuilder {
    private val waypoints = arrayListOf<Waypoint>()

    fun addWaypoint(point: Waypoint): PathBuilder {
        waypoints.add(point)
        return this
    }

    fun build(): Path {
        return Path(waypoints)
    }
}