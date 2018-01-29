package frc.team2186.robot.autonomous

import frc.team2186.robot.lib.common.actionRunner
import frc.team2186.robot.lib.interfaces.SequentialAutonomousMode
import frc.team2186.robot.subsystems.Drive

class DoNothing : SequentialAutonomousMode("Do Nothing", true) {
    override val actions = actionRunner {
        action {
            Drive.stop()

            Drive.stopped
        }
    }
}