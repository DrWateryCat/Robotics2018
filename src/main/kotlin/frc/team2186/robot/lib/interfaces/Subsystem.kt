package frc.team2186.robot.lib.interfaces

import frc.team2186.robot.lib.common.thread

abstract class Subsystem {
    init {
        thread {
            while (Thread.currentThread().isInterrupted.not()) {
                update()
            }
        }
    }

    abstract fun update()

    fun accessSync(block: () -> Unit) {
        synchronized(this) {
            block()
        }
    }
}