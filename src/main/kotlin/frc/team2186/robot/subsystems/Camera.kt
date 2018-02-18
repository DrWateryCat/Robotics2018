package frc.team2186.robot.subsystems

import edu.wpi.cscore.MjpegServer
import edu.wpi.cscore.UsbCamera
import edu.wpi.first.wpilibj.CameraServer
import frc.team2186.robot.Config
import frc.team2186.robot.lib.interfaces.Subsystem

object Camera : Subsystem() {
    private val cam: UsbCamera? = CameraServer.getInstance().startAutomaticCapture(Config.Camera.cameraID)
    private val server: MjpegServer? = CameraServer.getInstance().addServer("main_camera", Config.Camera.cameraPort).apply {
        source = cam
    }
    override fun update() {
    }
}