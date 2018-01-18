package frc.team2186.robot.autonomous

import frc.team2186.robot.lib.interfaces.AutonomousMode
import frc.team2186.robot.lib.math.Rotation2D
import frc.team2186.robot.subsystems.Drive

class PIDTuning : AutonomousMode() {
    override fun init() {
        Drive.useGyro = false
    }

    override fun update() {
        Drive.leftSetpoint = Drive.rpmToInchesPerSecond(60.0)
        Drive.rightSetpoint = Drive.rpmToInchesPerSecond(60.0)
    }

    override fun done(): Boolean = false
}