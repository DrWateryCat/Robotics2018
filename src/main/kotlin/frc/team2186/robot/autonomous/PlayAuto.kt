package frc.team2186.robot.autonomous

import com.google.gson.JsonElement
import com.google.gson.JsonParser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team2186.robot.lib.interfaces.AutonomousMode
import frc.team2186.robot.subsystems.Drive
import java.io.File
import java.io.FileInputStream
import java.io.InputStreamReader

class PlayAuto : AutonomousMode("Play a recorded auto", false) {
    private var autoData: File? = null
    private var fis: FileInputStream? = null
    private var json: JsonElement? = null
    private var canRun = true

    override fun done(): Boolean {
        return !canRun
    }

    override fun init() {
        try {
            autoData = File(SmartDashboard.getString("autofile", "auto-1.json"))
            fis = FileInputStream(autoData!!)
            json = JsonParser().parse(InputStreamReader(fis!!))

            Drive.useGyro = false
            Drive.useVelocityPid = true
        } catch (e: Exception) {
            System.err.println("Error loading auto file: " + e.message)
            System.err.println("Disabling auto")
            canRun = false
        }

    }

    override fun update() {
        try {
            val currentVelocities = json!!.asJsonObject.getAsJsonObject("" + deltaTime)
            val leftVel = currentVelocities.get("left_rpm").asDouble
            val rightVel = currentVelocities.get("right_rpm").asDouble

            Drive.leftSetpoint = leftVel
            Drive.rightSetpoint = rightVel
        } catch (e: Exception) {
            println("Error while playing: " + e.message)
            canRun = false
        }

    }
}
