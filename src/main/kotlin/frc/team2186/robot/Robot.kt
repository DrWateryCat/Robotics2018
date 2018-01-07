package frc.team2186.robot

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.IterativeRobot
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team2186.robot.autonomous.DoNothing
import frc.team2186.robot.autonomous.PIDTuning
import frc.team2186.robot.common.RobotPosition
import frc.team2186.robot.common.RobotState
import frc.team2186.robot.common.ScaleState
import frc.team2186.robot.common.SwitchState
import frc.team2186.robot.lib.interfaces.AutonomousMode
import frc.team2186.robot.lib.odometry.RobotPoseEstimator
import frc.team2186.robot.subsystems.Drive

class Robot : IterativeRobot() {
    val autoChooser = SendableChooser<AutonomousMode>()
    val positionChooser = SendableChooser<RobotPosition>()

    val leftJoystick = Joystick(Config.Controls.leftJoystickID)
    val rightJoystick = Joystick(Config.Controls.rightJoystickID)
    override fun robotInit() {
        Drive

        Thread {
            RobotPoseEstimator.run()
        }.start()

        autoChooser.addDefault("Do Nothing", DoNothing())
        autoChooser.addObject("Tune PID", PIDTuning())

        SmartDashboard.putData("Autonomous Mode", autoChooser)

        positionChooser.addDefault("Left", RobotPosition.LEFT)
        positionChooser.addObject("Middle", RobotPosition.MIDDLE)
        positionChooser.addObject("Right", RobotPosition.RIGHT)

        SmartDashboard.putData("Robot Position", positionChooser)

        val switchScaleSettings = DriverStation.getInstance().gameSpecificMessage
        StartingSwitch = (if (switchScaleSettings[0] == 'L') SwitchState.LEFT else SwitchState.RIGHT)
        StartingScale = (if (switchScaleSettings[1] == 'L') ScaleState.LEFT else ScaleState.RIGHT)
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

    override fun disabledPeriodic() {
        StartingPosition = positionChooser.selected
    }

    companion object {
        var CurrentMode = RobotState.DISABLED
        var StartingSwitch = SwitchState.LEFT
        var StartingScale = ScaleState.LEFT
        var StartingPosition = RobotPosition.LEFT
    }
}