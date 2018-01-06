package frc.team2186.robot.autonomous

import frc.team2186.robot.lib.interfaces.AutonomousMode
import frc.team2186.robot.subsystems.Drive

class PIDTuning : AutonomousMode() {
    override fun init() {
    }

    override fun update() {
        if (deltaTime > 5.0) {
            Drive.stop()
        } else {
            Drive.leftSetpoint = Drive.rpmToInchesPerSecond(60.0)
            Drive.rightSetpoint = Drive.rpmToInchesPerSecond(60.0)
        }
    }
}