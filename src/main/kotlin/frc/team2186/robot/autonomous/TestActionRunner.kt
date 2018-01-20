package frc.team2186.robot.autonomous

import frc.team2186.robot.lib.common.IterativeAutoAction
import frc.team2186.robot.lib.common.SequentialActionRunner
import frc.team2186.robot.lib.interfaces.AutonomousMode
import frc.team2186.robot.subsystems.Drive
import frc.team2186.robot.subsystems.Lifter
import frc.team2186.robot.subsystems.Manipulator

class TestActionRunner : AutonomousMode() {
    val runner = SequentialActionRunner(
            IterativeAutoAction {
                Drive.accessSync {
                    Drive.leftSetpoint = Drive.rpmToInchesPerSecond(60.0)
                    Drive.rightSetpoint = Drive.rpmToInchesPerSecond(60.0)
                }
                Lifter.accessSync {
                    Lifter.set(0.5)
                }

                Drive.leftPosition >= 60.0 && Drive.rightPosition >= 60.0 && Lifter.done
            },
            IterativeAutoAction {
                Lifter.accessSync {
                    Lifter.set(0.0)
                }
                Lifter.done
            }
    )
    override fun init() {
        Lifter.usePID = true
    }

    override fun update() = runner.update()

    override fun done(): Boolean = runner.done
}