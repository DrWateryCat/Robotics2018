package frc.team2186.robot.autonomous

import frc.team2186.robot.lib.interfaces.AutonomousMode
import frc.team2186.robot.subsystems.Drive

class TunePID : AutonomousMode("Tuning PID", false) {
    override fun init() {
        Drive.useVelocityPid = true
        Drive.useGyro = true
        println("Initializing")
    }

    override fun update() {
        Drive.setForwardRPM(60.0)
    }

    override fun done() = false

}