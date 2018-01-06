package frc.team2186.robot.lib.interfaces

abstract class Subsystem {
    init {
        Thread {
            while (Thread.interrupted().not()) {
                update()
            }
        }.start()
    }

    abstract fun update()
}