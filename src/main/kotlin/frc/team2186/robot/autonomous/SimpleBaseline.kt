package frc.team2186.robot.autonomous

import frc.team2186.robot.Robot
import frc.team2186.robot.common.RobotPosition
import frc.team2186.robot.lib.common.IterativeAutoAction
import frc.team2186.robot.lib.common.SequentialActionRunner
import frc.team2186.robot.lib.common.actionRunner
import frc.team2186.robot.lib.interfaces.AutonomousMode
import frc.team2186.robot.subsystems.Drive

class SimpleBaseline : AutonomousMode() {
    private val runner = actionRunner {
        action {
            Drive.leftSetpoint = 24.0 //2 feet per second
            Drive.rightSetpoint = 24.0

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

    override fun done() = runner.done

    override fun init() {
    }

    override fun update() {
        runner.update()
    }
}