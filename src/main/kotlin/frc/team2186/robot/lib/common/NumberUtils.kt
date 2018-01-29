@file:JvmName("NumberUtils")
package frc.team2186.robot.lib.common

fun withinRange(value: Double, lowerBound: Double, upperBound: Double) = value in lowerBound..upperBound

infix fun Pair<Double, Double>.inRange(other: Pair<Double, Double>): Boolean {
    return withinRange(this.first, other.first, other.second) and withinRange(this.second, other.first, other.second)
}