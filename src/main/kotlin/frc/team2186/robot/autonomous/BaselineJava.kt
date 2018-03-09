package frc.team2186.robot.autonomous

import frc.team2186.robot.lib.common.actionRunner
import frc.team2186.robot.lib.interfaces.AutonomousMode

class BaselineJava : AutonomousMode("Baseline in Java", false) {
    val actions = actionRunner {
        action {
            distance(80.0)()
        }
    }

    override fun init() {
        actions.init()
        println(actions)
    }

    override fun update() {
        actions.update()
    }

    override fun done() = actions.done
}
