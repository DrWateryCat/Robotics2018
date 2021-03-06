package frc.team2186.robot.autonomous

import frc.team2186.robot.Robot
import frc.team2186.robot.common.RobotPosition
import frc.team2186.robot.common.SwitchState
import frc.team2186.robot.lib.common.actionRunner
import frc.team2186.robot.lib.interfaces.SequentialAutonomousMode

class AdvancedSwitch : SequentialAutonomousMode("Advanced Switch", false) {
    override val actions = actionRunner {
        when (Robot.StartingPosition) {
            RobotPosition.LEFT -> {
                if (Robot.StartingSwitch === SwitchState.RIGHT) {
                    action(heading90())
                    action(cross(false))
                    action(heading270())
                    action(baseline(false))
                    action(dropBox())
                }
            }
            RobotPosition.MIDDLE -> {
                if (Robot.StartingSwitch === SwitchState.LEFT) {
                    action(heading270())
                    action(cross(true))
                    action(heading90())
                    action(baseline(false))
                    action(dropBox())
                } else {
                    action(heading90())
                    action(cross(true))
                    action(heading270())
                    action(baseline(false))
                    action(dropBox())
                }
            }
            RobotPosition.RIGHT -> {
                if (Robot.StartingSwitch === SwitchState.LEFT) {
                    action(heading270())
                    action(cross(false))
                    action(heading90())
                    action(baseline(false))
                    action(dropBox())
                }
            }
        }
    }
}
