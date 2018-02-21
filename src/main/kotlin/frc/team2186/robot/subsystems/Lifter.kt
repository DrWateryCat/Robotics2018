package frc.team2186.robot.subsystems

import edu.wpi.first.wpilibj.VictorSP
import frc.team2186.robot.Config
import frc.team2186.robot.lib.interfaces.Subsystem

object Lifter : Subsystem() {
    private val master = VictorSP(Config.Platform.lifterMaster)
    private val slave = VictorSP(Config.Platform.lifterSlave)
    var setpoint = 0.0
    override fun update() {
        master.set(setpoint)
        slave.set(setpoint)
    }
}