package frc.team2186.robot.lib.common

class IterativeAutoAction(private val block: () -> Boolean) {
    var done = false

    fun run() {
        if (done.not()) {
            if(block()) {
                done = true
            }
        }
    }
}

class SequentialActionRunner(vararg action: IterativeAutoAction) {
    var actions = action
    var index = 0

    val done get() = index > actions.size - 1
    fun update() {
        if (actions[index].done) {
            if ((index == actions.size - 1).not()) {
                index++
            }
        } else {
            actions[index].run()
        }
    }
}