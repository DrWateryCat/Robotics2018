package frc.team2186.robot.lib.interfaces

import com.google.gson.JsonObject
import frc.team2186.robot.lib.common.thread

abstract class Subsystem {
    /*
    init {
        thread {
            while (Thread.currentThread().isInterrupted.not()) {
                update()
            }
        }
    }*/

    abstract val json: JsonObject

    abstract fun update()

    fun accessSync(block: Subsystem.() -> Unit) {
        synchronized(this) {
            this.apply(block)
        }
    }
}