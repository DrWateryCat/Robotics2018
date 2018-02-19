package frc.team2186.robot.subsystems

import frc.team2186.robot.Config
import frc.team2186.robot.common.CANMotor
import frc.team2186.robot.common.Motor
import frc.team2186.robot.lib.interfaces.Subsystem

object Grabber : Subsystem() {
    private val left = Motor(Config.Grabber.Left).apply {
        inverted = true
    }
    private val right = CANMotor(Config.Grabber.Right)

    var setpoint = 0.0

    val hasBox: Boolean
        get() = left.isStalled and right.isStalled

    override fun update() {
        left.set(setpoint)
        right.set(setpoint)
    }
}