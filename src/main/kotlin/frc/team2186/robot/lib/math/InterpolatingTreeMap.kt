package frc.team2186.robot.lib.math

import frc.team2186.robot.lib.interfaces.Interpolable
import frc.team2186.robot.lib.interfaces.InverseInterpolable
import java.util.*

class InterpolatingTreeMap<K, V>(val max: Int = 0): TreeMap<K, V>()
        where K: InverseInterpolable<K>, K: Comparable<K>,  V: Interpolable<V>{
    override fun put(key: K, value: V): V {
        if (max in 1..size) {
            remove(firstKey())
        }
        super.put(key, value)
        return value
    }

    override fun putAll(from: Map<out K, V>) {
        println("Don't use this function!")
    }

    fun getInterpolated(key: K): V? {
        val gotVal = get(key)

        if (gotVal == null) {
            val topBound = ceilingKey(key)
            val bottomBound = floorKey(key)

            when {
                topBound == null && bottomBound == null -> return null
                topBound == null -> return get(bottomBound)
                bottomBound == null -> return get(topBound)
            }

            val top = get(topBound)
            val bot = get(bottomBound)
            return bot!!.interpolate(bot, bottomBound.inverseInterpolate(topBound, key))
        } else {
            return gotVal
        }
    }
}