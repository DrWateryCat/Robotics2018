package frc.team2186.robot.subsystems

import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.VictorSP
import frc.team2186.robot.Config
import frc.team2186.robot.lib.common.IterativeAutoAction
import frc.team2186.robot.lib.interfaces.Subsystem

object Platform : Subsystem() {
    private val motor = VictorSP(Config.Platform.motorID)
    private val barDown = DigitalInput(Config.Platform.barDownSwitch)
    private val barUp = DigitalInput(Config.Platform.barUpSwitch)

    var setpoint = 0.0

    val isBarDown
        get() = barDown.get()

    val isBarUp
        get() = barUp.get()

    fun barUp() = IterativeAutoAction {
            setpoint = 0.25

            isBarUp
    }

    fun barDown() = IterativeAutoAction {
        setpoint = -0.25
        isBarDown
    }


    override fun update() {
        motor.set(when {
            isBarDown.not() or (setpoint >= 0.0) -> setpoint
            isBarUp.not() or (setpoint <= 0.0) -> setpoint
            else -> 0.0
        })
    }
}