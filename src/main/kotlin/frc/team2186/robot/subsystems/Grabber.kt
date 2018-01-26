package frc.team2186.robot.subsystems

import com.google.gson.JsonObject
import frc.team2186.robot.lib.interfaces.Subsystem

object Grabber : Subsystem() {
    override val json: JsonObject
        @Synchronized
        get() = JsonObject().apply {

        }
    override fun update() {
    }
}