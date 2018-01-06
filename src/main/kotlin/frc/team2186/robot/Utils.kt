package frc.team2186.robot

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX

operator fun WPI_TalonSRX.plus(slave: WPI_TalonSRX): WPI_TalonSRX = apply {
    follow(this)
}