package frc.team2186.robot.subsystems

import com.google.gson.JsonObject
import edu.wpi.first.wpilibj.SerialPort
import frc.team2186.robot.lib.interfaces.Subsystem

object Lights : Subsystem() {
    enum class Animations {
        RAINBOW, RED_ALLIANCE, BLUE_ALLIANCE
    }
    private var lastAnimation = Animations.RAINBOW
    var animation = Animations.RAINBOW

    private val json: JsonObject
        get() = JsonObject().apply {
            addProperty("animation", when(animation) {
                Animations.RAINBOW -> 0
                Animations.RED_ALLIANCE -> 1
                Animations.BLUE_ALLIANCE -> 2
            })
        }

    private val serial = SerialPort(9600, SerialPort.Port.kMXP)

    override fun update() {
        if (lastAnimation != animation) {
            lastAnimation = animation
            serial.writeString(json.toString())
        }
    }
}