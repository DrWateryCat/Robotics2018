package frc.team2186.robot

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.IterativeRobot
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.PowerDistributionPanel
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team2186.robot.autonomous.*
import frc.team2186.robot.common.*
import frc.team2186.robot.lib.common.pressedButtons
import frc.team2186.robot.lib.interfaces.AutonomousMode
import frc.team2186.robot.lib.networking.EasyNetworkTable
import frc.team2186.robot.lib.odometry.Kinematics
import frc.team2186.robot.subsystems.*

class Robot : IterativeRobot() {
    var autoChooser = SendableChooser<AutonomousMode>()
    val positionChooser = SendableChooser<RobotPosition>()
    val pathChooser = SendableChooser<RecordingManager.Paths>()

    val leftJoystick = Joystick(Config.Controls.leftJoystickID)
    val rightJoystick = Joystick(Config.Controls.rightJoystickID)

    val networkTable = EasyNetworkTable("/robot")

    val subsystems = arrayListOf(
            Drive,
            RobotPoseEstimator,
            Grabber,
            Platform,
            RecordingManager
    )

    override fun robotInit() {
        println("Intializing")
        Drive
        Grabber
        Platform
        Camera
        RobotPoseEstimator
        RecordingManager


        Kinematics.apply {
            wheelDiameter = Config.Drive.wheelDiameter
            effectiveWheelDiameter = Config.Drive.effectiveWheelDiameter
            trackScrubFactor = Config.Drive.trackScrubFactor
        }
        autoChooser.apply {
            /*
            Reflections("frc.team2186.robot.autonomous").getSubTypesOf(AutonomousMode::class.java).forEach {
                val i = it.newInstance()
                if (i.default) {
                    addDefault(i.name, i)
                } else {
                    addObject(i.name, i)
                }
            }*/
            addDefault("Do Nothing", DoNothing())
            addObject("Baseline", BaselineJava())
            addObject("Switch", Switch())
            addObject("Tune PID", TunePID())
            addObject("Pure Pursuit", PurePursuitAuto())
            addObject("Play Auto", PlayPathAuto())
        }

        pathChooser.apply {
            RecordingManager.Paths.values().forEach {
                addObject(it.name, it)
            }
        }
        SmartDashboard.putData("paths", pathChooser)

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
        SelectedPath = pathChooser.selected!!
        autoChooser.selected.init()
        Drive.reset()
    }

    override fun autonomousPeriodic() {
        if (autoChooser.selected.done().not()) {
            autoChooser.selected.update()
        }
        periodic()
    }

    override fun teleopInit() {
        CurrentMode = RobotState.TELEOP
        Drive.reset()
    }

    override fun teleopPeriodic() {
        Drive.tankDrive(-leftJoystick.getRawAxis(1), rightJoystick.getRawAxis(1))
        Platform.setpoint = when {
            leftJoystick.getRawButton(Config.Controls.lifterUpButton) -> -0.5
            rightJoystick.getRawButton(Config.Controls.lifterUpButton) -> -0.5
            else -> 0.0
        }

        Grabber.leftSetpoint = when {
            leftJoystick.getRawButton(4) or leftJoystick.getRawButton(1) -> 1.0
            leftJoystick.getRawButton(5) or rightJoystick.getRawButton(1) -> -1.0
            else -> 0.0
        }
        Grabber.rightSetpoint = when {
            rightJoystick.getRawButton(5) or leftJoystick.getRawButton(1) -> 1.0
            rightJoystick.getRawButton(4) or rightJoystick.getRawButton(1) -> -1.0
            else -> 0.0
        }
        leftJoystick.pressedButtons().forEach {
            when(it) {
                8 -> {
                    RecordingManager.record()
                }
                9 -> {
                    RecordingManager.stop()
                }
                10 -> {
                    RecordingManager.save()
                }
            }
        }
        networkTable.apply {
            putNumber("time", DriverStation.getInstance().matchTime.toInt())
        }
        periodic()
    }

    override fun disabledInit() {
        CurrentMode = RobotState.DISABLED
    }

    override fun disabledPeriodic() {
        periodic()
    }

    fun periodic() {
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
        var SelectedPath = RecordingManager.Paths.LeftSwitchLeft

        val pdp = PowerDistributionPanel(20)
    }
}