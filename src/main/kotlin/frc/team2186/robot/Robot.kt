package frc.team2186.robot

import edu.wpi.first.wpilibj.IterativeRobot
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team2186.robot.autonomous.DoNothing
import frc.team2186.robot.common.RobotState
import frc.team2186.robot.lib.interfaces.AutonomousMode
import frc.team2186.robot.lib.odometry.RobotPoseEstimator
import frc.team2186.robot.subsystems.Drive

class Robot : IterativeRobot() {
    val autoChooser = SendableChooser<AutonomousMode>()

    val leftJoystick = Joystick(Config.Controls.leftJoystickID)
    val rightJoystick = Joystick(Config.Controls.rightJoystickID)
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
        Drive.leftSetpoint = leftJoystick.getRawAxis(1)
        Drive.rightSetpoint = rightJoystick.getRawAxis(1)
    }

    override fun disabledInit() {
        CurrentMode = RobotState.DISABLED
    }

    companion object {
        var CurrentMode = RobotState.DISABLED
    }
}