package frc.team2186.robot.autonomous

import frc.team2186.robot.Robot
import frc.team2186.robot.common.RobotPosition
import frc.team2186.robot.lib.common.IterativeAutoAction
import frc.team2186.robot.lib.common.SequentialActionRunner
import frc.team2186.robot.lib.interfaces.AutonomousMode
import frc.team2186.robot.lib.math.Translation2D
import frc.team2186.robot.lib.pathfinding.Path
import frc.team2186.robot.lib.pathfinding.Waypoint
import frc.team2186.robot.subsystems.Drive

class Baseline : AutonomousMode("Baseline") {
    val runner = SequentialActionRunner(
            IterativeAutoAction {
                Drive.followPath(when(Robot.StartingPosition) {
                    RobotPosition.LEFT -> {
                        Path(
                                Waypoint(Translation2D(0.0, 0.0), 0.0, "Start"),
                                Waypoint(Translation2D(-0.5 * 12, 5.0 * 12), 40.0, "Middle"),
                                Waypoint(Translation2D(-0.5 * 12, 5.0 * 12), 0.0, "End")
                        )
                    }
                    RobotPosition.MIDDLE -> {
                        Path(
                                Waypoint(Translation2D(0.0, 0.0), 0.0, "Start"),
                                Waypoint(Translation2D(-5.0 * 12, 12.0), 40.0),
                                Waypoint(Translation2D(-5.0 * 12, 4.0 * 12), 40.0),
                                Waypoint(Translation2D(-5.0 * 12, 6.0 * 12), 0.0, "End")
                        )
                    }
                    RobotPosition.RIGHT -> {
                        Path(
                                Waypoint(Translation2D(0.0, 0.0), 0.0, "Start")
                        )
                    }
                })
                true
            },
            IterativeAutoAction {
                Drive.finishedPath
            }
    )
    override fun init() {
        Drive.useGyro = true
    }

    override fun update() {
        runner.update()
    }

    override fun done(): Boolean = runner.done
}