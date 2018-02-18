package frc.team2186.robot

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.IterativeRobot
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.PowerDistributionPanel
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team2186.robot.common.RobotPosition
import frc.team2186.robot.common.RobotState
import frc.team2186.robot.common.ScaleState
import frc.team2186.robot.common.SwitchState
import frc.team2186.robot.lib.interfaces.AutonomousMode
import frc.team2186.robot.lib.odometry.Kinematics
import frc.team2186.robot.subsystems.*
import org.reflections.Reflections

class Robot : IterativeRobot() {
    var autoChooser: SendableChooser<AutonomousMode>? = null
    val positionChooser = SendableChooser<RobotPosition>()

    val leftJoystick = Joystick(Config.Controls.leftJoystickID)
    val rightJoystick = Joystick(Config.Controls.rightJoystickID)
    val codriver = Joystick(Config.Controls.codriverJoystickID)

    val subsystems = arrayListOf(
            Drive,
            RobotPoseEstimator,
            Grabber.getInstance(),
            Platform,
            Lifter
    )

    override fun robotInit() {
        Drive
        RobotPoseEstimator
        Platform
        Grabber.getInstance()
        Lifter
        //Camera

        Kinematics.apply {
            wheelDiameter = Config.Drive.wheelDiameter
            effectiveWheelDiameter = Config.Drive.effectiveWheelDiameter
            trackScrubFactor = Config.Drive.trackScrubFactor
        }
        autoChooser = SendableChooser()

        autoChooser?.apply {
            try {
                Reflections("frc.team2186.robot.autonomous").getSubTypesOf(AutonomousMode::class.java).forEach { it ->
                    val c = it.newInstance()
                    if (c.default) {
                        addDefault(c.name, c)
                    } else {
                        addObject(c.name, c)
                    }
                }
            } catch (e: Exception) {
                e.printStackTrace()
                println("Disabling autonomous.")
                autoChooser = null
            }
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
        autoChooser?.selected?.init() ?: println("Attempted to initialize the selected autonomous, but it was null!")

        CurrentMode = RobotState.AUTONOMOUS
    }

    override fun autonomousPeriodic() {
        //autoChooser?.selected?.update()
        SmartDashboard.putString("current_auto", autoChooser?.selected?.name ?: "None")
    }

    override fun teleopInit() {
        CurrentMode = RobotState.TELEOP
    }

    override fun teleopPeriodic() {
        Drive.accessSync {
            Drive.leftSetpoint = leftJoystick.getRawAxis(1)
            Drive.rightSetpoint = rightJoystick.getRawAxis(1)
        }
        Platform.setpoint = when {
            codriver.getRawButton(Config.Controls.lifterUpButton) -> 0.25
            codriver.getRawButton(Config.Controls.lifterDownButton) -> -0.25
            else -> 0.0
        }
        Grabber.getInstance().setpoint = when {
            codriver.getRawButton(Config.Controls.grabberInButton) -> -0.25
            codriver.getRawButton(Config.Controls.grabberOutButton) -> 0.25
            else -> 0.0
        }
        Lifter.setpoint = codriver.getRawAxis(1)
        subsystems.forEach {
            it.update()
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

        val pdp = PowerDistributionPanel(20)
    }
}