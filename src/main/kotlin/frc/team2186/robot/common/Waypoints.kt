package frc.team2186.robot.common

import com.google.gson.JsonObject
import com.google.gson.JsonParser
import frc.team2186.robot.Config
import frc.team2186.robot.lib.math.Translation2D
import java.io.File

object Waypoints {
    private val waypoints: JsonObject
        get() = JsonParser()
                .parse(File(Config.PathFollowing.waypointsFile.toURI())
                        .bufferedReader())
                .asJsonObject

    private fun getFromPosition(robotPosition: RobotPosition): JsonObject {
        return waypoints.getAsJsonObject("starting").getAsJsonObject(when(robotPosition) {
            RobotPosition.LEFT -> "left"
            RobotPosition.MIDDLE -> "middle"
            RobotPosition.RIGHT -> "right"
        })
    }

    fun switch(robotPosition: RobotPosition, switchState: SwitchState): Translation2D {
        val waypoint = getFromPosition(robotPosition).getAsJsonObject(when(switchState) {
            SwitchState.LEFT -> "left"
            SwitchState.RIGHT -> "right"
        })

        return Translation2D(waypoint["x"].asDouble, waypoint["y"].asDouble)
    }

    fun scale(robotPosition: RobotPosition, scaleState: ScaleState): Translation2D {
        val waypoint = getFromPosition(robotPosition).getAsJsonObject(when (scaleState) {
            ScaleState.LEFT -> "left"
            ScaleState.RIGHT -> "right"
        })

        return Translation2D(waypoint["x"].asDouble, waypoint["y"].asDouble)
    }

    val baseline: Translation2D
        get() {
            val waypoint = waypoints["baseline"].asJsonObject
            return Translation2D(waypoint["x"].asDouble, waypoint["y"].asDouble)
        }
}