package frc.team2186.robot.subsystems

import edu.wpi.first.wpilibj.VictorSP
import frc.team2186.robot.Config
import frc.team2186.robot.lib.interfaces.Subsystem

object Lifter : Subsystem() {
    private val motor = VictorSP(Config.Platform.lifterID)
    var setpoint = 0.0
        set(value) {
            setpoint = 0.25 * value
        }
    override fun update() {
        motor.set(setpoint)
    }
}