package frc.team2186.robot.subsystems

import frc.team2186.robot.Robot
import frc.team2186.robot.lib.interfaces.Subsystem
import frc.team2186.robot.lib.networking.EasyNetworkTable
import frc.team2186.robot.lib.odometry.FramesOfReference

object DashboardUpdater : Subsystem() {
    val odometry = EasyNetworkTable("/odometry")
    val drive = EasyNetworkTable("/drive")
    val robotState = EasyNetworkTable("/robot_state")

    override fun update() {
        odometry.apply {
            putString("pose_json", FramesOfReference.latestFieldToVehicle().value.json.asString)
        }
        
        Drive.accessSync {
            drive.apply {
                putString("drive_json", Drive.json.asString)
            }
        }

        robotState.apply {
            putString("selected_position", Robot.CurrentMode.name)
            putString("scale_value", Robot.StartingScale.name)
            putString("switch_scale", Robot.StartingSwitch.name)
            putString("starting_position", Robot.StartingPosition.name)
        }
    }
}