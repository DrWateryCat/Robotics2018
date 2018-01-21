package frc.team2186.robot.autonomous

import frc.team2186.robot.lib.common.actionRunner
import frc.team2186.robot.lib.interfaces.AutonomousMode
import frc.team2186.robot.subsystems.Drive
import frc.team2186.robot.subsystems.Lifter

class TestActionRunner : AutonomousMode() {
    val runner = actionRunner {
        init {
            Drive.reset()
        }
        action {
            Drive.leftSetpoint = Drive.rpmToInchesPerSecond(60.0)
            Drive.rightSetpoint = Drive.rpmToInchesPerSecond(60.0)

            Drive.leftPosition >= 60.0 && Drive.rightPosition >= 60.0
        }
        actionComplete {
            Drive.reset()
        }
    }
    override fun init() {
        Lifter.usePID = true
        runner.init()
    }

    override fun update() = runner.update()

    override fun done(): Boolean = runner.done
}