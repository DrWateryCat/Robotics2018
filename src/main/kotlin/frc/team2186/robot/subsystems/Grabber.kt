package frc.team2186.robot.subsystems

import com.google.gson.JsonObject
import edu.wpi.first.wpilibj.PowerDistributionPanel
import edu.wpi.first.wpilibj.SpeedController
import edu.wpi.first.wpilibj.VictorSP
import frc.team2186.robot.Config
import frc.team2186.robot.lib.interfaces.Subsystem

object Grabber : Subsystem() {
    class Motor(val config: Config.Grabber.Motor) {
        val motor = VictorSP(config.id)
        val stalled: Boolean
            get() = current >= config.peakCurrent

        val current: Double
            get() = pdp.getCurrent(config.pdpChannel)

        fun set(value: Double) {
            motor.set(value)
        }

        var inverted: Boolean = false
            set(value) { motor.inverted = value }
    }
    val pdp = PowerDistributionPanel(20)
    val left = Motor(Config.Grabber.Left)
    val right = Motor(Config.Grabber.Right).apply {
        inverted = true
    }

    @get:Synchronized
    override val json: JsonObject
        get() = JsonObject().apply {
            addProperty("has_box", hasBox)
            addProperty("left_current", left.current)
            addProperty("right_current", right.current)
        }

    @get:Synchronized
    val hasBox: Boolean
        get() = left.stalled and right.stalled

    @set:Synchronized
    @get:Synchronized
    var value = 0.0

    override fun update() {
        left.set(value)
        right.set(value)
    }
}