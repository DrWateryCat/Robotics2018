package frc.team2186.robot.autonomous

import frc.team2186.robot.lib.common.IterativeAutoAction
import frc.team2186.robot.lib.common.SequentialActionRunner
import frc.team2186.robot.lib.interfaces.AutonomousMode
import frc.team2186.robot.subsystems.Drive
import frc.team2186.robot.subsystems.Manipulator

class TestActionRunner : AutonomousMode() {
    val runner = SequentialActionRunner(
            IterativeAutoAction {
                Drive.leftSetpoint = Drive.rpmToInchesPerSecond(60.0)
                Drive.rightSetpoint = Drive.rpmToInchesPerSecond(60.0)

                Drive.leftPosition >= 60.0 && Drive.rightPosition >= 60.0
            },
            IterativeAutoAction {
                Manipulator.setpoint = 0.5
                Manipulator.currentPosition in 0.5..0.55
            }
    )
    override fun init() {
    }

    override fun update() {
        runner.update()
    }

    override fun done(): Boolean = runner.done
}