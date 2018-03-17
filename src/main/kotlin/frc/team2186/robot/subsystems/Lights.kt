package frc.team2186.robot.subsystems

import com.google.gson.Gson
import frc.team2186.robot.Config
import frc.team2186.robot.lib.common.thread
import frc.team2186.robot.lib.interfaces.Subsystem
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress

object Lights : Subsystem() {
    enum class Animation {
        OFF,
        MANUAL,
        RAINBOW,
        RAINBOW_CYCLE
    }
    private data class ControlData(val key: String, val value: Int)
    private val gson = Gson()

    private var animation: Animation = Animation.OFF
        set(value) {
            if (field != value) {
                field = value
                updateController(ControlData("animation", value.ordinal))
            }
        }

    private var red = 0
        set(value) {
            if (field != value) {
                field = value
                if(animation != Animation.MANUAL) {
                    animation = Animation.MANUAL
                }
                updateController(ControlData("red", value))
            }
        }

    private var green = 0
        set(value) {
            if (field != value) {
                field = value
                if(animation != Animation.MANUAL) {
                    animation = Animation.MANUAL
                }
                updateController(ControlData("green", value))
            }
        }

    private var blue = 0
        set(value) {
            if(field != value) {
                field = value
                if(animation != Animation.MANUAL) {
                    animation = Animation.MANUAL
                }
                updateController(ControlData("blue", value))
            }
        }

    fun blueAlliance() {
        animation = Animation.MANUAL
        red = 0
        green = 0
        blue = 255
    }

    fun redAlliance() {
        animation = Animation.MANUAL
        red = 255
        green = 0
        blue = 0
    }

    fun rainbow() {
        animation = Animation.RAINBOW
    }

    fun rainbowCycle() {
        animation = Animation.RAINBOW_CYCLE
    }

    private fun updateController(data: ControlData) = thread {
        try {
            val outputData = gson.toJson(data, ControlData::class.java).toByteArray()

            val address = InetAddress.getByName(Config.Lights.ip)
            val sock = DatagramSocket()

            val sendPacket = DatagramPacket(outputData, outputData.size, address, Config.Lights.port)
            sock.send(sendPacket)
        } catch (e: Exception) {
            e.printStackTrace()
            println("Did not send packet to raspberry pi")
        }
    }

    override fun update() {
    }
}