package frc.team2186.robot.lib.interfaces

interface InverseInterpolable<T> {
    fun inverseInterpolate(upper: T, lower: T): Double
}
