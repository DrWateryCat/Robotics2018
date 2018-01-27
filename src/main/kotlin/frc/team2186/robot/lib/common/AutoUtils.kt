@file:JvmName("AutoUtils")
package frc.team2186.robot.lib.common

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import frc.team2186.robot.lib.interfaces.AutonomousMode

class IterativeAutoAction(val block: () -> Boolean) {
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

class ActionRunner {
    private val actions = ArrayList<IterativeAutoAction>()
    private val actionCompletedCallbacks = ArrayList<() -> Unit>()
    private lateinit var initBlock: () -> Unit
    val done get() = (actions.size > 0).not()

    fun action(block: () -> Boolean) = actions.add(IterativeAutoAction(block))
    fun actionComplete(block: () -> Unit) = actionCompletedCallbacks.add(block)
    fun init(block: () -> Unit) {initBlock = block}

    fun init() {
        this.initBlock()
    }

    fun update() {
        if (actions[0].done.and(done.not())) {
            if (actions.size > 0) {
                actions.removeAt(0)
                actionCompletedCallbacks.forEach { cb ->
                    cb()
                }
            }
        } else {
            actions[0].run()
        }
    }
}

object AutonomousManager {
    private val modes = ArrayList<AutonomousMode>()

    fun addMode(mode: AutonomousMode) {
        modes.add(mode)
    }

    fun sendableChooser(): SendableChooser<AutonomousMode> {
        return SendableChooser<AutonomousMode>().apply {
            modes.forEach {
                if (it.default) {
                    addDefault(it.name, it)
                } else {
                    addObject(it.name, it)
                }
            }
        }
    }
}

fun actionRunner(block: ActionRunner.() -> Unit): ActionRunner = ActionRunner().apply(block)