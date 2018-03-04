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

    var leftSetpoint = 0.0
    var rightSetpoint = 0.0

    val hasBox: Boolean
        get() = left.isStalled and right.isStalled

    override fun update() {
        left.set(leftSetpoint * 0.25)
        right.set(rightSetpoint * 0.25)
    }
}