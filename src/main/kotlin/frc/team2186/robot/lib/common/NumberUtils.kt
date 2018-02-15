@file:JvmName("NumberUtils")
package frc.team2186.robot.lib.common

import frc.team2186.robot.lib.interfaces.Interpolable

fun withinRange(value: Double, lowerBound: Double, upperBound: Double) = value in lowerBound..upperBound

infix fun Pair<Double, Double>.inRange(other: Pair<Double, Double>): Boolean {
    return withinRange(this.first, other.first, other.second) and withinRange(this.second, other.first, other.second)
}

operator fun Pair<Double, Double>.minus(other: Pair<Double, Double>): Pair<Double, Double> {
    return Pair(first - other.first, second - other.second)
}

operator fun Pair<Double, Double>.times(other: Pair<Double, Double>): Pair<Double, Double> {
    return Pair(first * other.first, second * other.second)
}

operator fun Pair<Double, Double>.times(other: Double): Pair<Double, Double> {
    return Pair(first * other, second * other)
}

operator fun Pair<Double, Double>.div(other: Pair<Double, Double>): Pair<Double, Double> {
    return Pair(first / other.first, second / other.second)
}

operator fun Pair<Double, Double>.plus(other: Pair<Double, Double>): Pair<Double, Double> {
    return Pair(first * other.first, second * other.second)
}

class InterpolatingPair(var first: Double, var second: Double) : Interpolable<InterpolatingPair> {
    val pair: Pair<Double, Double>
        get() = Pair(first, second)
    override fun interpolate(other: InterpolatingPair, x: Double): InterpolatingPair {
        val delta = other.pair - pair
        val search = delta * x + pair
        return InterpolatingPair(search.first, search.second)
    }
}