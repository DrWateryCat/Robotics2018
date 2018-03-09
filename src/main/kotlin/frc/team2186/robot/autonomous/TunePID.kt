package frc.team2186.robot.autonomous

import frc.team2186.robot.lib.interfaces.AutonomousMode
import frc.team2186.robot.subsystems.Drive

class TunePID : AutonomousMode("Tuning PID", false) {
    override fun init() {
        println("Initializing")
    }

    override fun update() {
        Drive.setForwardVelocity(Drive.rpmToInchesPerSecond(60.0))
    }

    override fun done() = false

}