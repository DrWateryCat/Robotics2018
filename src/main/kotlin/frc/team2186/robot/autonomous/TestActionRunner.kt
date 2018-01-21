package frc.team2186.robot.autonomous

import frc.team2186.robot.lib.common.ActionRunner
import frc.team2186.robot.lib.common.actionRunner
import frc.team2186.robot.lib.interfaces.AutonomousMode
import frc.team2186.robot.lib.interfaces.SequentialAutonomousMode
import frc.team2186.robot.subsystems.Drive
import frc.team2186.robot.subsystems.Lifter

class TestActionRunner : SequentialAutonomousMode("Action Runner Test") {
    override val actions: ActionRunner = actionRunner {
        init {
            Drive.reset()
        }
        action {
            Drive.setForwardRPM(60.0)
            Lifter.set(0.5)

            Drive.leftPosition >= 60.0 && Drive.rightPosition >= 60.0 && Lifter.done
        }
        actionComplete {
            Drive.reset()
            Lifter.stop()
        }
    }
    override fun init() {
        Lifter.usePID = true
        super.init()
    }
}