package frc.team2186.robot.autonomous

import frc.team2186.robot.autonomous.CommonAuto.baseline
import frc.team2186.robot.autonomous.CommonAuto.stop
import frc.team2186.robot.lib.common.actionRunner
import frc.team2186.robot.lib.interfaces.SequentialAutonomousMode

class BaselineJava : SequentialAutonomousMode("Baseline in Java", false) {
    override val actions = actionRunner {
        baseline(false)
        stop()
    }
}
