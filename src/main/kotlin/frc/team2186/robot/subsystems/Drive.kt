package frc.team2186.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.SerialPort
import frc.team2186.robot.lib.interfaces.Subsystem
import frc.team2186.robot.Config
import frc.team2186.robot.Robot
import frc.team2186.robot.common.RobotState
import frc.team2186.robot.common.SynchronousPID
import frc.team2186.robot.lib.math.Rotation2D
import frc.team2186.robot.plus
import kotlin.math.PI

object Drive : Subsystem() {
    private data class DriveData(val left: Double, val right: Double)

    private val leftSide = WPI_TalonSRX(Config.Drive.leftMasterID).apply {
        configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0)
    } + WPI_TalonSRX(Config.Drive.leftSlaveID).apply {
    }

    private val rightSide = WPI_TalonSRX(Config.Drive.rightMasterID).apply {
        configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0)
        inverted = true
    } + WPI_TalonSRX(Config.Drive.rightSlaveID).apply {
    }

    private val gyro = AHRS(SerialPort.Port.kMXP)

    private val velocityHeadingPID = SynchronousPID(1.0, 0.0, 0.0)

    val leftPosition: Double get() = ticksToInches(leftSide.getSelectedSensorPosition(0).toDouble())
    val rightPosition: Double get() = ticksToInches(rightSide.getSelectedSensorPosition(0).toDouble())

    val leftVelocity: Double get() = ticksToInchesPerSecond(leftSide.getSelectedSensorVelocity(0).toDouble())
    val rightVelocity: Double get() = ticksToInchesPerSecond(rightSide.getSelectedSensorVelocity(0).toDouble())

    val gyroAngle: Rotation2D get() = Rotation2D.fromDegrees(gyro.yaw.toDouble())

    var leftSetpoint: Double = 0.0
    var rightSetpoint: Double = 0.0
    var gyroSetpoint: Double = 0.0

    fun inchesPerSecondToRPM(ips: Double): Double = ips * 60 / (Config.Drive.wheelDiameter * PI)
    fun rpmToTicks(rpm: Double): Double = rpm * Config.Drive.ticksPerRevolution / (1 / 60 / 100)
    fun inchesPerSecondToTicks(ips: Double): Double = rpmToTicks(inchesPerSecondToRPM(ips))

    fun ticksToRPM(ticks: Double): Double = ticks * Config.Drive.ticksPerRevolution * (1 / 60 / 100)
    fun rpmToInchesPerSecond(rpm: Double): Double = rpm * (Config.Drive.ticksPerRevolution * PI) / 60
    fun ticksToInchesPerSecond(ticks: Double): Double = rpmToInchesPerSecond(ticksToRPM(ticks))

    init {
        velocityHeadingPID.setOutputRange(-30.0, 30.0)
    }

    fun ticksToInches(ticks: Double): Double {
        return ticks / Config.Drive.ticksPerRevolution * (Config.Drive.wheelDiameter * PI)
    }

    fun inchesToTicks(inches: Double): Double {
        return inches * Config.Drive.ticksPerRevolution / (Config.Drive.wheelDiameter * PI)
    }

    fun reset() {
        leftSide.setSelectedSensorPosition(0, 0, 0)
        rightSide.setSelectedSensorPosition(0, 0, 0)
    }

    fun stop() {
        leftSetpoint = 0.0
        rightSetpoint = 0.0

        leftSide.stopMotor()
        rightSide.stopMotor()
    }

    private fun updateVelocityHeading(): DriveData {
        val currentGyro = gyroAngle
        val lastHeadingError = Rotation2D.fromDegrees(gyroSetpoint).rotateBy(currentGyro.inverse()).degrees

        val deltaV = velocityHeadingPID.calculate(lastHeadingError)

        return DriveData(leftSetpoint + deltaV / 2, rightSetpoint + deltaV / 2)
    }

    override fun update() {
        when {
            Robot.CurrentMode == RobotState.DISABLED -> {
                leftSide.set(ControlMode.PercentOutput, 0.0)
                rightSide.set(ControlMode.PercentOutput, 0.0)
            }

            Robot.CurrentMode == RobotState.TELEOP -> {
                leftSide.set(ControlMode.PercentOutput, leftSetpoint)
                rightSide.set(ControlMode.PercentOutput, rightSetpoint)
            }

            Robot.CurrentMode == RobotState.AUTONOMOUS -> {
                val command = updateVelocityHeading()

                leftSide.set(ControlMode.Velocity, command.left)
                rightSide.set(ControlMode.Velocity, command.right)
            }
        }
    }
}