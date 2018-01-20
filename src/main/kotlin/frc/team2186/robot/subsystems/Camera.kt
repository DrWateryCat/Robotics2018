package frc.team2186.robot.subsystems

import com.google.gson.JsonObject
import edu.wpi.first.wpilibj.CameraServer
import frc.team2186.robot.Config
import frc.team2186.robot.lib.interfaces.Subsystem

object Camera : Subsystem() {
    val cam = CameraServer.getInstance().startAutomaticCapture(Config.Camera.cameraID)
    val server = CameraServer.getInstance().addServer("main_camera", Config.Camera.cameraPort)

    override val json: JsonObject
        get() = JsonObject().apply {
        }
    init {
        server.source = cam
    }
    override fun update() {
    }
}