package frc.team2186.robot

import com.ctre.phoenix.motorcontrol.IMotorController
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX

import frc.team2186.robot.lib.interfaces.Subsystem

operator fun WPI_TalonSRX.plus(slave: WPI_TalonSRX): WPI_TalonSRX = apply {
    follow(this)
}

class TalonSRX(val talonSRX: TalonSRX): IMotorController by talonSRX {}