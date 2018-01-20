package frc.team2186.robot.lib.common

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX

operator fun WPI_TalonSRX.plus(slave: WPI_TalonSRX): WPI_TalonSRX = apply {
    slave.follow(this)
}

operator fun WPI_TalonSRX.plus(slave: WPI_VictorSPX): WPI_TalonSRX = apply {
    slave.follow(this)
}