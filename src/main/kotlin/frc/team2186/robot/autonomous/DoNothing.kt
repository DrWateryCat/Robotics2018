package frc.team2186.robot.autonomous

import frc.team2186.robot.lib.common.actionRunner
import frc.team2186.robot.lib.interfaces.SequentialAutonomousMode

class DoNothing : SequentialAutonomousMode("Do Nothing", true) {
    override val actions = actionRunner {
        action {
            println("Doing nothing")
            true
        }
    }
}
