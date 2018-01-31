@file:JvmName("MotorUtils")
package frc.team2186.robot.lib.common

import com.ctre.phoenix.motorcontrol.IMotorController
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX
import edu.wpi.first.wpilibj.PIDSource
import edu.wpi.first.wpilibj.PIDSourceType

typealias CANTalon = WPI_TalonSRX
typealias CANVictor = WPI_VictorSPX

operator fun CANTalon.plus(slave: CANTalon) = apply {
    slave.follow(this)
}

operator fun CANTalon.plus(slave: CANVictor) = apply {
    slave.follow(this)
}

operator fun IMotorController.plus(slave: IMotorController) = apply {
    slave.follow(this)
}

class PIDInput(val block: () -> Double): PIDSource {
    var sourceType = PIDSourceType.kDisplacement
    override fun getPIDSourceType() = sourceType
    override fun setPIDSourceType(pidSource: PIDSourceType?) { sourceType = pidSource ?: sourceType }

    @Synchronized
    override fun pidGet() = block()
}