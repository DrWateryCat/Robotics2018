package frc.team2186.robot.lib.math

object Utils {
    fun epsilonEquals(a: Double, b: Double, epsilon: Double): Boolean = a - epsilon <= b && a + epsilon >= b
}
