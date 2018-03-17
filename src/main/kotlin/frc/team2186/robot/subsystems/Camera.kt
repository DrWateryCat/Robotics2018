package frc.team2186.robot.subsystems

import edu.wpi.first.wpilibj.CameraServer
import frc.team2186.robot.lib.interfaces.Subsystem

object Camera : Subsystem() {
    private val camera = CameraServer.getInstance().startAutomaticCapture(0).apply {
        setResolution(320, 240)
        setFPS(30)
    }
    private val server = CameraServer.getInstance().addServer("server0", 5801).apply {
        source = camera
    }

    init {
    }
    override fun update() {
    }
}