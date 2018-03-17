package frc.team2186.robot.autonomous

import com.sun.prism.impl.Disposer
import frc.team2186.robot.Robot
import frc.team2186.robot.common.RecordingManager
import frc.team2186.robot.common.RobotPosition
import frc.team2186.robot.common.SwitchState
import frc.team2186.robot.lib.common.actionRunner
import frc.team2186.robot.lib.interfaces.SequentialAutonomousMode
import frc.team2186.robot.subsystems.Drive
import frc.team2186.robot.subsystems.Platform

class AutoPath : SequentialAutonomousMode("Path", false) {
    override val actions = actionRunner {
        init {
            RecordingManager.play(startingPos())
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

    private fun startingPos() = when(Robot.StartingPosition) {
        RobotPosition.LEFT -> {
            if (Robot.StartingSwitch == SwitchState.RIGHT) {
                RecordingManager.Paths.LeftSwitchRight
            } else {
                RecordingManager.Paths.LeftSwitchLeft
            }
        }
        RobotPosition.MIDDLE -> {
            if (Robot.StartingSwitch == SwitchState.RIGHT) {
                RecordingManager.Paths.MiddleSwitchRight
            } else {
                RecordingManager.Paths.MiddleSwitchLeft
            }
        }
        RobotPosition.RIGHT -> {
            if (Robot.StartingSwitch == SwitchState.RIGHT) {
                RecordingManager.Paths.RightSwitchRight
            } else {
                RecordingManager.Paths.RightSwitchLeft
            }
        }
    }
}