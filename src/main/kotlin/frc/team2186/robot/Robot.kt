package frc.team2186.robot

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.IterativeRobot
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.PowerDistributionPanel
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team2186.robot.autonomous.*
import frc.team2186.robot.common.RobotPosition
import frc.team2186.robot.common.RobotState
import frc.team2186.robot.common.ScaleState
import frc.team2186.robot.common.SwitchState
import frc.team2186.robot.lib.interfaces.AutonomousMode
import frc.team2186.robot.lib.networking.EasyNetworkTable
import frc.team2186.robot.lib.odometry.Kinematics
import frc.team2186.robot.subsystems.*

class Robot : IterativeRobot() {
    var autoChooser = SendableChooser<AutonomousMode>()
    val positionChooser = SendableChooser<RobotPosition>()

    val leftJoystick = Joystick(Config.Controls.leftJoystickID)
    val rightJoystick = Joystick(Config.Controls.rightJoystickID)
    val codriver = Joystick(Config.Controls.codriverJoystickID)

    val networkTable = EasyNetworkTable("/robot")

    val subsystems = arrayListOf(
            Drive,
            RobotPoseEstimator,
            Grabber,
            Platform,
            Lifter
    )

    override fun robotInit() {
        println("Intializing")
        Drive

        Kinematics.apply {
            wheelDiameter = Config.Drive.wheelDiameter
            effectiveWheelDiameter = Config.Drive.effectiveWheelDiameter
            trackScrubFactor = Config.Drive.trackScrubFactor
        }
        autoChooser.apply {
            addDefault("Do Nothing", DoNothing())
            addObject("Baseline", BaselineJava())
            addObject("Play auto", PlayAuto())
            addObject("Switch", Switch())
            addObject("Tune PID", TunePID())
            addObject("Time test", TimeTest())
        }

        SmartDashboard.putData("autonomous", autoChooser)

        positionChooser.apply {
            addDefault("Left", RobotPosition.LEFT)
            addObject("Middle", RobotPosition.MIDDLE)
            addObject("Right", RobotPosition.RIGHT)
        }

        SmartDashboard.putData("position", positionChooser)
    }

    override fun autonomousInit() {
        updateSwitchScale()
        autoChooser.selected.init()

        CurrentMode = RobotState.AUTONOMOUS
        Drive.inAuto = true
        Drive.reset()
        println(CurrentMode)
    }

    override fun autonomousPeriodic() {
        if (autoChooser.selected.done().not()) {
            autoChooser.selected.update()
        }

        subsystems.forEach {
            it.update()
        }
    }

    override fun teleopInit() {
        CurrentMode = RobotState.TELEOP
        Drive.inAuto = false
        Drive.reset()
    }

    override fun teleopPeriodic() {
        Drive.accessSync {
            Drive.leftSetpoint = leftJoystick.getRawAxis(1)
            Drive.rightSetpoint = rightJoystick.getRawAxis(1)
        }
        Platform.setpoint = when {
            leftJoystick.getRawButton(Config.Controls.lifterUpButton) -> -0.5
            rightJoystick.getRawButton(Config.Controls.lifterUpButton) -> -0.5
            else -> 0.0
        }
        Grabber.setpoint = when {
            leftJoystick.getRawButton(Config.Controls.grabberButton) -> -0.25
            rightJoystick.getRawButton(Config.Controls.grabberButton) -> 0.25
            else -> 0.0
        }
        Lifter.setpoint = codriver.getRawAxis(1)
        subsystems.forEach {
            it.update()
        }

        networkTable.apply {
            putNumber("time", DriverStation.getInstance().matchTime.toInt())
        }
    }

    override fun disabledInit() {
        CurrentMode = RobotState.DISABLED
        Drive.inAuto = false
    }

    override fun disabledPeriodic() {
        subsystems.forEach {
            it.update()
        }
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

        val pdp = PowerDistributionPanel(20)
    }
}