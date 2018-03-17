package frc.team2186.robot.autonomous

import frc.team2186.robot.Robot
import frc.team2186.robot.common.RecordingManager
import frc.team2186.robot.lib.common.actionRunner
import frc.team2186.robot.lib.interfaces.SequentialAutonomousMode
import frc.team2186.robot.subsystems.Drive
import frc.team2186.robot.subsystems.Lights
import frc.team2186.robot.subsystems.Platform

class PlayPathAuto : SequentialAutonomousMode("Play path", false) {
    override val actions = actionRunner {
        init {
            RecordingManager.play(Robot.SelectedPath)
            Lights.rainbowCycle()
        }
        action {
            RecordingManager.finished
        }
        action {
            Platform.setpoint = -0.5
            Platform.isBarUp
        }
        action {
            Platform.setpoint = 0.0
            Drive.tankDrive(0.0, 0.0)
            true
        }
    }
}