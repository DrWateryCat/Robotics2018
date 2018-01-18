package frc.team2186.robot

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.IterativeRobot
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team2186.robot.autonomous.DoNothing
import frc.team2186.robot.autonomous.PIDTuning
import frc.team2186.robot.autonomous.TestFollowPath
import frc.team2186.robot.common.RobotPosition
import frc.team2186.robot.common.RobotState
import frc.team2186.robot.common.ScaleState
import frc.team2186.robot.common.SwitchState
import frc.team2186.robot.lib.interfaces.AutonomousMode
import frc.team2186.robot.lib.odometry.RobotPoseEstimator
import frc.team2186.robot.subsystems.DashboardUpdater
import frc.team2186.robot.subsystems.Drive
import frc.team2186.robot.subsystems.Manipulator

class Robot : IterativeRobot() {
    val autoChooser = SendableChooser<AutonomousMode>()
    val positionChooser = SendableChooser<RobotPosition>()

    val leftJoystick = Joystick(Config.Controls.leftJoystickID)
    val rightJoystick = Joystick(Config.Controls.rightJoystickID)
    override fun robotInit() {
        Drive
        DashboardUpdater
        RobotPoseEstimator
        Manipulator

        autoChooser.apply {
            addDefault("Do Nothing", DoNothing())
            addObject("Tune PID", PIDTuning())
            addObject("Following a path test", TestFollowPath())
        }

        SmartDashboard.putData("autonomous", autoChooser)

        positionChooser.apply {
            addDefault("Left", RobotPosition.LEFT)
            addObject("Middle", RobotPosition.MIDDLE)
            addObject("Right", RobotPosition.RIGHT)
        }

        SmartDashboard.putData("Robot Position", positionChooser)
    }

    override fun autonomousInit() {
        updateSwitchScale()
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
        Drive.accessSync {
            Drive.leftSetpoint = leftJoystick.getRawAxis(1)
            Drive.rightSetpoint = rightJoystick.getRawAxis(1)
        }
    }

    override fun disabledInit() {
        CurrentMode = RobotState.DISABLED
    }

    override fun disabledPeriodic() {
    }

    fun updateSwitchScale() {
        StartingPosition = positionChooser.selected
        val switchScaleSettings = DriverStation.getInstance().gameSpecificMessage
        StartingSwitch = when {
            switchScaleSettings[0] == 'L' -> SwitchState.LEFT
            switchScaleSettings[0] == 'R' -> SwitchState.RIGHT
            else -> {
                println("HIT A BAD MODE! Switching current switch mode to left.")
                SwitchState.LEFT
            }
        }
        StartingScale = when {
            switchScaleSettings[1] == 'L' -> ScaleState.LEFT
            switchScaleSettings[1] == 'R' -> ScaleState.RIGHT
            else -> {
                println("HIT A BAD MODE! Switching current scale mode to left.")
                ScaleState.LEFT
            }
        }
    }

    companion object {
        var CurrentMode = RobotState.DISABLED
        var StartingSwitch = SwitchState.LEFT
        var StartingScale = ScaleState.LEFT
        var StartingPosition = RobotPosition.LEFT
    }
}