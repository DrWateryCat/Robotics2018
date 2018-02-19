package frc.team2186.robot.common

import edu.wpi.first.wpilibj.PowerDistributionPanel
import edu.wpi.first.wpilibj.SpeedController
import edu.wpi.first.wpilibj.VictorSP
import frc.team2186.robot.Config
import frc.team2186.robot.Robot

class Motor(cfg: Config.Grabber.Motor) : SpeedController {
    private val motor: VictorSP = VictorSP(cfg.id)
    private val pdp: PowerDistributionPanel = Robot.pdp

    private val pdpChannel = cfg.pdpChannel
    private val peakCurrent = cfg.peakCurrent

    private var setpoint = 0.0

    val current: Double
        get() = pdp.getCurrent(pdpChannel)

    val isStalled: Boolean
        get() = current >= peakCurrent

    override fun set(speed: Double) {
        setpoint = speed
        motor.set(setpoint)
    }

    override fun get(): Double {
        return setpoint
    }

    override fun setInverted(isInverted: Boolean) {
        motor.inverted = isInverted
    }

    override fun getInverted(): Boolean {
        return motor.inverted
    }

    override fun disable() {
        motor.disable()
    }

    override fun stopMotor() {
        motor.stopMotor()
    }

    override fun pidWrite(output: Double) {
        setpoint = output
        motor.pidWrite(output)
    }
}
