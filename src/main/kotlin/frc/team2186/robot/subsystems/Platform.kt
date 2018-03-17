package frc.team2186.robot.subsystems

import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.VictorSP
import frc.team2186.robot.Config
import frc.team2186.robot.lib.common.IterativeAutoAction
import frc.team2186.robot.lib.interfaces.Subsystem
import frc.team2186.robot.lib.networking.EasyNetworkTable

object Platform : Subsystem() {
    private val motor = VictorSP(Config.Platform.motorID)
    private val switch = DigitalInput(0)

    private val networkTable = EasyNetworkTable("/platform")

    var setpoint = 0.0

    val isBarUp
        get() = switch.get().not()

    fun barUp() = IterativeAutoAction {
        setpoint = -0.25
        isBarUp
    }

    fun barDown() = IterativeAutoAction {
        setpoint = 0.0
        true
    }

    override fun update() {
        motor.set(when {
            isBarUp.not() or (setpoint <= 0.0) -> setpoint
            else -> 0.0
        })

        networkTable.apply {
            putBoolean("barUp", isBarUp, false)
        }
    }
}