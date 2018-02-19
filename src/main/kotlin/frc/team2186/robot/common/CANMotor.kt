package frc.team2186.robot.common

import com.ctre.phoenix.motorcontrol.ControlMode
import frc.team2186.robot.Config
import frc.team2186.robot.lib.common.CANVictor

class CANMotor(cfg: Config.Grabber.Motor) {
    private val motor = CANVictor(cfg.id)
    private val peakCurrent = cfg.peakCurrent

    val isStalled: Boolean
        get() = motor.outputCurrent >= peakCurrent

    val current: Double
        get() = motor.outputCurrent

    fun set(value: Double) {
        motor.set(ControlMode.PercentOutput, value)
    }
}