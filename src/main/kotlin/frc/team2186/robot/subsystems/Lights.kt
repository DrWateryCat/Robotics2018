package frc.team2186.robot.subsystems

import com.google.gson.JsonObject
import com.google.gson.JsonParser
import edu.wpi.first.wpilibj.SerialPort
import frc.team2186.robot.lib.interfaces.Subsystem

object Lights : Subsystem() {
    enum class Animations {
        RAINBOW, RED_ALLIANCE, BLUE_ALLIANCE, MANUAL
    }
    var red: Int = 0
        @Synchronized
        set(value) {
            field = value
        }
    var green = 0
        @Synchronized
        set(value) {
            field = value
        }
    var blue = 0
        @Synchronized
        set(value) {
            field = value
        }
    var animation = Animations.RAINBOW
        @Synchronized
        set(value) {
            field = value
        }

    override val json: JsonObject
        @Synchronized
        get() = JsonObject().apply {
            addProperty("animation", when(animation) {
                Animations.RAINBOW -> 0
                Animations.RED_ALLIANCE -> 1
                Animations.BLUE_ALLIANCE -> 2
                Animations.MANUAL -> 3
            })
            addProperty("red", red)
            addProperty("green", green)
            addProperty("blue", blue)
        }

    private val serial = SerialPort(9600, SerialPort.Port.kUSB).apply {
        enableTermination('\n')
    }

    override fun update() {
        serial.writeString(json.asString)
    }
}