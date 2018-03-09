package frc.team2186.robot.autonomous

import frc.team2186.robot.lib.interfaces.AutonomousMode
import frc.team2186.robot.subsystems.Drive
import frc.team2186.robot.subsystems.Platform

class TimeTest : AutonomousMode("Time Test", false) {
    override fun done() = deltaTime > 6.0

    override fun init() {
    }

    override fun update() {
        Drive.setForwardVelocity(-20.0)

        if (deltaTime >= 5.0) {
            Drive.tankDrive(0.0, 0.0)

            Platform.setpoint = -0.5
            if (deltaTime >= 5.5) {
                Platform.setpoint = 0.0
            }
        }
    }
}