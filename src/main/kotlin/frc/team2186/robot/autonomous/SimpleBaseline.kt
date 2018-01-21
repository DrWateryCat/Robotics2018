package frc.team2186.robot.autonomous

import frc.team2186.robot.lib.common.actionRunner
import frc.team2186.robot.lib.interfaces.SequentialAutonomousMode
import frc.team2186.robot.subsystems.Drive

class SimpleBaseline : SequentialAutonomousMode() {
    override val actions = actionRunner {
        action {
            Drive.setForwardVelocity(24.0) //2 feet per second

            Drive.leftPosition >= 60.0 && Drive.rightPosition >= 60.0
        }
        action {
            Drive.stop()
            Drive.leftVelocity == 0.0 && Drive.rightVelocity == 0.0
        }
        actionComplete {
            Drive.reset()
        }
    }
}