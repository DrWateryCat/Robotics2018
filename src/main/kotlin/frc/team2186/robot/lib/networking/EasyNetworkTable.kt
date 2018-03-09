package frc.team2186.robot.lib.networking

import edu.wpi.first.networktables.NetworkTableInstance

class EasyNetworkTable (path: String){
    data class Data(val key: String, val value: Any)
    private var table = NetworkTableInstance.getDefault().getTable(path)

    @Synchronized
    fun getEntry(key: String) = table.getEntry(key)

    @Synchronized
    fun getString(key: String, defaultVal: String):String {
        return getEntry(key).getString(defaultVal)
    }

    @Synchronized
    fun getNumber(key: String, defaultVal: Number): Number {
        return getEntry(key).getNumber(defaultVal)
    }

    @Synchronized
    fun putString(key: String, value: String) {
        this += Data(key, value)
    }

    @Synchronized
    fun putNumber(key: String, value: Number) {
        this += Data(key, value)
    }

    @Synchronized
    fun putBoolean(key: String, value: Boolean, default: Boolean = false) {
        getEntry(key).setValue(value)
        getEntry(key).setDefaultBoolean(default)
    }

    @Synchronized
    fun put(data: Data) {
        this += data
    }

    @Synchronized
    operator fun plusAssign(d: Data) {
        getEntry(d.key).setValue(d.value)
    }

    operator fun set(key: String, value: Any) {
        getEntry(key).setValue(value)
    }

    operator fun get(key: String): Any? = getEntry(key).value.value
}