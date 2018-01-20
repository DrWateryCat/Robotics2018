package frc.team2186.robot.subsystems

import com.google.gson.JsonObject
import frc.team2186.robot.Robot
import frc.team2186.robot.lib.interfaces.Subsystem
import frc.team2186.robot.lib.networking.EasyNetworkTable
import frc.team2186.robot.lib.odometry.FramesOfReference
import frc.team2186.robot.lib.odometry.RobotPoseEstimator

object DashboardUpdater : Subsystem() {
    val odometry = EasyNetworkTable("/odometry")
    val drive = EasyNetworkTable("/drive")
    val lifter = EasyNetworkTable("/lifter")
    val robotState = EasyNetworkTable("/robot_state")

    override val json: JsonObject
        get() = JsonObject().apply {
        }

    fun updateNetworkTable(subsystem: Subsystem, networkTable: EasyNetworkTable) {
        subsystem.accessSync {
            this.json.entrySet().forEach {
                networkTable.putString(it.key, it.value.asString)
            }
        }
    }

    override fun update() {
        updateNetworkTable(RobotPoseEstimator, odometry)
        updateNetworkTable(Drive, drive)
        updateNetworkTable(Lifter, lifter)

        robotState.apply {
            putString("selected_position", Robot.CurrentMode.name)
            putString("scale_value", Robot.StartingScale.name)
            putString("switch_scale", Robot.StartingSwitch.name)
            putString("starting_position", Robot.StartingPosition.name)
        }
    }
}