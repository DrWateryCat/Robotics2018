package frc.team2186.robot

import edu.wpi.first.wpilibj.IterativeRobot
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team2186.robot.autonomous.DoNothing
import frc.team2186.robot.common.RobotState
import frc.team2186.robot.lib.interfaces.AutonomousMode
import frc.team2186.robot.lib.odometry.RobotPoseEstimator
import frc.team2186.robot.subsystems.Drive

class Robot : IterativeRobot() {
    val autoChooser = SendableChooser<AutonomousMode>()
    override fun robotInit() {
        Drive

        Thread {
            RobotPoseEstimator.run()
        }.start()

        autoChooser.addDefault("Do Nothing", DoNothing())

        SmartDashboard.putData("Autonomous Mode", autoChooser)
    }

    override fun autonomousInit() {
        autoChooser.selected.init()

        CurrentMode = RobotState.AUTONOMOUS
    }

    override fun autonomousPeriodic() {
        autoChooser.selected.update()
    }

    override fun teleopInit() {
        CurrentMode = RobotState.TELEOP
    }

    override fun teleopPeriodic() {
    }

    override fun disabledInit() {
        CurrentMode = RobotState.DISABLED
    }

    companion object {
        var CurrentMode = RobotState.DISABLED
    }
}