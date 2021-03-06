package frc.team2186.robot.lib.pathfinding

class Lookahead(val minDistance: Double, val maxDistance: Double, val minSpeed: Double, val maxSpeed: Double) {
    val deltaX = maxDistance - minDistance
    val deltaV = maxSpeed - minSpeed

    fun getLookaheadForSpeed(speed: Double): Double{
        val lookahead = deltaX * (speed - minSpeed) / deltaV + minDistance
        return when (lookahead) {
            Double.NaN -> minDistance
            else -> Math.max(minDistance, Math.min(maxDistance, lookahead))
        }
    }
}