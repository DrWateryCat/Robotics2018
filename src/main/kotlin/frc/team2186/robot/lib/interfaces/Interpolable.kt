package frc.team2186.robot.lib.interfaces

interface Interpolable<T> {
    fun interpolate(other: T, x: Double): T
}
