package frc.team2186.robot.subsystems

import frc.team2186.robot.lib.interfaces.Subsystem
import frc.team2186.robot.lib.networking.EasyNetworkTable
import frc.team2186.robot.lib.odometry.FramesOfReference

object DashboardUpdater : Subsystem() {
    val odometry = EasyNetworkTable("/odometry")
    val drive = EasyNetworkTable("/drive")

    override fun update() {
        val currentPose = FramesOfReference.latestFieldToVehicle().value

        odometry.apply {
            putString("pose_json", currentPose.json.asString)
        }
        
        drive.apply { 
            putString("drive_json", Drive.json.asString)
        }
    }
}