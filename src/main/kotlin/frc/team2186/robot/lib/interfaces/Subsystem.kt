package frc.team2186.robot.lib.interfaces

abstract class Subsystem {
    /*
    init {
        thread {
            while (Thread.currentThread().isInterrupted.not()) {
                update()
            }
        }
    }*/
    abstract fun update()

    fun accessSync(block: Subsystem.() -> Unit) {
        synchronized(this) {
            this.apply(block)
        }
    }
}