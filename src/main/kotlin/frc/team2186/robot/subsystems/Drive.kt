package frc.team2186.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.google.gson.JsonObject
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.*
import frc.team2186.robot.lib.interfaces.Subsystem
import frc.team2186.robot.Config
import frc.team2186.robot.Robot
import frc.team2186.robot.common.RobotState
import frc.team2186.robot.lib.common.*
import frc.team2186.robot.lib.math.Rotation2D
import frc.team2186.robot.lib.odometry.FramesOfReference
import frc.team2186.robot.lib.odometry.Kinematics
import frc.team2186.robot.lib.pathfinding.Path
import frc.team2186.robot.lib.pathfinding.PurePursuitController
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.max

object Drive : Subsystem(){
    private data class DriveData(val left: Double, val right: Double)

    private val leftSide = CANTalon(Config.Drive.leftMasterID).apply {
        configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0)
        config_kP(0, Config.Drive.kLeftP, 0)
        config_kI(0, Config.Drive.kLeftI, 0)
        config_kD(0, Config.Drive.kLeftD, 0)
        config_kF(0, Config.Drive.kLeftF, 0)
    } + CANVictor(Config.Drive.leftSlaveID).apply {
    }

    private val rightSide = CANTalon(Config.Drive.rightMasterID).apply {
        configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0)
        config_kP(0, Config.Drive.kRightP, 0)
        config_kI(0, Config.Drive.kRightI, 0)
        config_kD(0, Config.Drive.kRightD, 0)
        config_kF(0, Config.Drive.kRightF, 0)
        inverted = true
    } + CANVictor(Config.Drive.rightSlaveID).apply {
    }

    private val pidInterface = PIDInput {
        gyroSetpoint.rotateBy(gyroAngle.inverse()).degrees
    }

    private val gyro = AHRS(SerialPort.Port.kMXP)
    private val velocityHeadingPID = PIDController(Config.Drive.kHeadingP,
                                                   Config.Drive.kHeadingI,
                                                   Config.Drive.kHeadingD,
                                                   pidInterface,
                                                   PIDOutput {
        deltaV = it
    })

    private lateinit var ppController: PurePursuitController

    private var followingPath = false

    @set:Synchronized
    private var deltaV = 0.0

    @set:Synchronized
    var useGyro = true

    @set:Synchronized
    var useVelocityPid = true

    @get:Synchronized
    val leftPosition: Double
        get() = ticksToInches(leftSide.getSelectedSensorPosition(0).toDouble())

    @get:Synchronized
    val rightPosition: Double
        get() = ticksToInches(rightSide.getSelectedSensorPosition(0).toDouble())

    @get:Synchronized
    val leftVelocity: Double
        get() = ticksToInchesPerSecond(leftSide.getSelectedSensorVelocity(0).toDouble())

    @get:Synchronized
    val rightVelocity: Double
        get() = ticksToInchesPerSecond(rightSide.getSelectedSensorVelocity(0).toDouble())

    @get:Synchronized
    val velocities: Pair<Double, Double>
        get() = Pair(leftVelocity, rightVelocity)

    @get:Synchronized
    val stopped: Boolean
        get() = velocities inRange Pair(-0.05, 0.05)

    @get:Synchronized
    val gyroAngle: Rotation2D
        get() = Rotation2D.fromDegrees(gyro.yaw.toDouble())

    @get:Synchronized
    val finishedPath
        get() = ppController.isDone

    @set:Synchronized
    var leftSetpoint: Double = 0.0

    @set:Synchronized
    var rightSetpoint: Double = 0.0

    @set:Synchronized
    var gyroSetpoint: Rotation2D = Rotation2D.fromDegrees(0.0)

    override val json get() = JsonObject().apply {
        addProperty("left_velocity", leftVelocity)
        addProperty("right_velocity", rightVelocity)
        addProperty("left_setpoint", leftSetpoint)
        addProperty("right_setpoint", rightSetpoint)
        addProperty("current_gyro", gyroAngle.degrees)
    }

    fun inchesPerSecondToRPM(ips: Double): Double = ips * 60 / (Config.Drive.wheelDiameter * PI)
    fun rpmToTicks(rpm: Double): Double = rpmToNative(rpm)
    fun inchesPerSecondToTicks(ips: Double): Double = rpmToTicks(inchesPerSecondToRPM(ips))

    fun ticksToRPM(ticks: Double): Double = nativeToRpm(ticks)
    fun rpmToInchesPerSecond(rpm: Double): Double = rpm * (Config.Drive.wheelDiameter * PI) / 60
    fun ticksToInchesPerSecond(ticks: Double): Double = rpmToInchesPerSecond(ticksToRPM(ticks))

    init {
        velocityHeadingPID.setOutputRange(-30.0, 30.0)
    }

    fun ticksToInches(ticks: Double): Double {
        return ticks / Config.Drive.ticksPerRevolution * (Config.Drive.wheelDiameter * PI)
    }

    fun inchesToTicks(inches: Double): Double {
        return (inches / (Config.Drive.wheelDiameter * PI)) * Config.Drive.ticksPerRevolution
    }

    fun rpmToNative(rpm: Double) = rpm * (Config.Drive.ticksPerRevolution / 600)
    fun nativeToRpm(native: Double) = native / (Config.Drive.ticksPerRevolution / 600)

    @Synchronized
    fun reset() {
        leftSide.setSelectedSensorPosition(0, 0, 0)
        rightSide.setSelectedSensorPosition(0, 0, 0)
    }

    @Synchronized
    fun setForwardVelocity(ips: Double) {
        leftSetpoint = ips
        rightSetpoint = ips
    }

    @Synchronized
    fun setForwardRPM(rpm: Double) {
        leftSetpoint = rpmToInchesPerSecond(rpm)
        rightSetpoint = rpmToInchesPerSecond(rpm)
    }

    @Synchronized
    fun stop() {
        useVelocityPid = false

        leftSetpoint = 0.0
        rightSetpoint = 0.0

        leftSide.stopMotor()
        rightSide.stopMotor()
    }

    @Synchronized
    fun followPath(path: Path, reversed: Boolean = false) {
        ppController = PurePursuitController(
                Config.PathFollowing.fixedLookahead,
                Config.PathFollowing.maxAccel,
                Config.PathFollowing.nominalDt,
                path,
                reversed,
                Config.PathFollowing.completionTolerance
        )

        followingPath = true
    }

    private fun updateVelocityHeading() = DriveData(leftSetpoint + deltaV / 2, rightSetpoint + deltaV / 2)

    private fun updatePathFollower(): DriveData {
        val currentPose = FramesOfReference.latestFieldToVehicle().value
        val command = ppController.update(currentPose, Timer.getFPGATimestamp())

        var setpoint = Kinematics.inverseKinematics(command)

        var maxVel = 0.0
        maxVel = max(maxVel, abs(setpoint.left))
        maxVel = max(maxVel, abs(setpoint.right))

        if (maxVel > Config.PathFollowing.maxVelocity) {
            val scaling = Config.PathFollowing.maxVelocity / maxVel
            setpoint = Kinematics.DriveVelocity(setpoint.left * scaling, setpoint.right * scaling)
        }

        return DriveData(setpoint.left, setpoint.right)
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
                val command = when {
                    useGyro.not() -> DriveData(leftSetpoint, rightSetpoint)
                    followingPath -> {
                        if (finishedPath)
                            followingPath = false
                        updatePathFollower()
                    }
                    useVelocityPid -> {
                        updateVelocityHeading()
                    }
                    else -> DriveData(leftSetpoint, rightSetpoint)
                }

                leftSide.set(if (useVelocityPid) ControlMode.Velocity else ControlMode.PercentOutput, command.left)
                rightSide.set(if (useVelocityPid) ControlMode.Velocity else ControlMode.PercentOutput, command.right)
            }
        }
    }
}