package frc.team2186.robot.autonomous

import frc.team2186.robot.lib.common.actionRunner
import frc.team2186.robot.lib.common.inRange
import frc.team2186.robot.lib.common.withinRange
import frc.team2186.robot.lib.interfaces.SequentialAutonomousMode
import frc.team2186.robot.subsystems.Drive

class SimpleBaseline : SequentialAutonomousMode("Simple Baseline") {
    override val actions = actionRunner {
        action {
            Drive.setForwardVelocity(24.0) //2 feet per second

            Drive.leftPosition >= 120.0 && Drive.rightPosition >= 120.0
        }
        action {
            Drive.stop()

            Drive.stopped
        }
        actionComplete {
            Drive.reset()
        }
    }
}