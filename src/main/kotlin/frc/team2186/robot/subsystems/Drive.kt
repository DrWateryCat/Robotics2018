package frc.team2186.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.google.gson.JsonObject
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.PIDController
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team2186.robot.Config
import frc.team2186.robot.Robot
import frc.team2186.robot.common.RobotState
import frc.team2186.robot.lib.common.*
import frc.team2186.robot.lib.interfaces.Subsystem
import frc.team2186.robot.lib.math.InterpolatingDouble
import frc.team2186.robot.lib.math.InterpolatingTreeMap
import frc.team2186.robot.lib.math.Rotation2D
import frc.team2186.robot.lib.networking.EasyNetworkTable
import frc.team2186.robot.lib.odometry.FramesOfReference
import frc.team2186.robot.lib.odometry.Kinematics
import frc.team2186.robot.lib.pathfinding.Path
import frc.team2186.robot.lib.pathfinding.PurePursuitController
import java.io.File
import kotlin.math.PI

object Drive : Subsystem(){
    private data class DriveData(val left: Double, val right: Double)

    private val leftSide = CANTalon(Config.Drive.leftMasterID).apply {
        configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0)
        config_kP(0, Config.Drive.kLeftP, 0)
        config_kI(0, Config.Drive.kLeftI, 0)
        config_kD(0, Config.Drive.kLeftD, 0)
        config_kF(0, Config.Drive.kLeftF, 0)
        enableVoltageCompensation(true)
    } + CANVictor(Config.Drive.leftSlaveID).apply {
        enableVoltageCompensation(true)
    }

    private val rightSide = CANTalon(Config.Drive.rightMasterID).apply {
        configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0)
        config_kP(0, Config.Drive.kRightP, 0)
        config_kI(0, Config.Drive.kRightI, 0)
        config_kD(0, Config.Drive.kRightD, 0)
        config_kF(0, Config.Drive.kRightF, 0)
        enableVoltageCompensation(true)
        inverted = true
    } + CANVictor(Config.Drive.rightSlaveID).apply {
        enableVoltageCompensation(true)
        inverted = true
    }

    private val pidInterface = PIDInput {
        gyroSetpoint.rotateBy(gyroAngle.inverse()).degrees
    }
    private val gyro = AHRS(SPI.Port.kMXP)
    private val velocityHeadingPID = PIDController(
            Config.Drive.kHeadingP,
            Config.Drive.kHeadingI,
            Config.Drive.kHeadingD,
            Config.Drive.kHeadingF,
            pidInterface
    ) {
        deltaV = it
    }

    private val recording = InterpolatingTreeMap<InterpolatingDouble, InterpolatingPair>()
    private var recordingFile: File? = null
    private var startTime = 0.0

    private var pathFollower: PurePursuitController? = null

    private val networkTable = EasyNetworkTable("/drive")

    @set:Synchronized
    var useGyro = true

    @set:Synchronized
    var useVelocityPid = true

    @set:Synchronized
    var positionPid = false

    @get:Synchronized
    var isRecording = false

    @get:Synchronized
    @set:Synchronized
    var deltaV = 0.0

    @get:Synchronized
    val leftPosition: Double
        get() = ticksToInches(-leftSide.getSelectedSensorPosition(0).toDouble())

    @get:Synchronized
    val rightPosition: Double
        get() = ticksToInches(rightSide.getSelectedSensorPosition(0).toDouble())

    @get:Synchronized
    val leftVelocity: Double
        get() = ticksToInchesPerSecond(-leftSide.getSelectedSensorVelocity(0).toDouble())

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
    val compassAngle: Rotation2D
        get() = Rotation2D.fromDegrees(gyro.compassHeading.toDouble())

    @get:Synchronized
    val fusedHeading: Rotation2D
        get() = Rotation2D.fromDegrees(gyro.fusedHeading.toDouble())

    @set:Synchronized
    var leftSetpoint: Double = 0.0

    @set:Synchronized
    var rightSetpoint: Double = 0.0

    @set:Synchronized
    var gyroSetpoint: Rotation2D = Rotation2D.fromDegrees(0.0)

    @get:Synchronized
    @set:Synchronized
    var inverted = false

    @set:Synchronized
    var inAuto = false

    @get:Synchronized
    val onTarget: Boolean
        get() = velocityHeadingPID.onTarget()

    fun inchesPerSecondToRPM(ips: Double): Double = ips * 60 / (Config.Drive.wheelDiameter * PI)
    fun rpmToTicks(rpm: Double): Double = rpmToNative(rpm)
    fun inchesPerSecondToTicks(ips: Double): Double = rpmToTicks(inchesPerSecondToRPM(ips))

    fun ticksToRPM(ticks: Double): Double = nativeToRpm(ticks)
    fun rpmToInchesPerSecond(rpm: Double): Double = rpm * (Config.Drive.wheelDiameter * PI) / 60
    fun ticksToInchesPerSecond(ticks: Double): Double = rpmToInchesPerSecond(ticksToRPM(ticks))

    init {
        velocityHeadingPID.setOutputRange(-30.0, 30.0)
        SmartDashboard.putData(leftSide)
        SmartDashboard.putData(rightSide)
        SmartDashboard.putData(velocityHeadingPID)
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
    fun record(filePath: String) {
        recordingFile = File(filePath)
        isRecording = true
        startTime = Timer.getFPGATimestamp()
    }

    @Synchronized
    fun stopRecording() {
        isRecording = false
        val outputData = JsonObject().apply {
            recording.forEach { k, v ->
                add("${k.value}", JsonObject().apply {
                    addProperty("left_rpm", v.first)
                    addProperty("right_rpm", v.second)
                })
            }
        }

        recordingFile?.printWriter()?.print(outputData.toString())
    }

    @Synchronized
    fun reset() {
        leftSide.setSelectedSensorPosition(0, 0, 0)
        rightSide.setSelectedSensorPosition(0, 0, 0)
        leftSide.setIntegralAccumulator(0.0, 0, 0)
        rightSide.setIntegralAccumulator(0.0, 0, 0)

        gyro.reset()
    }

    fun followPath(p: Path) {
        pathFollower = PurePursuitController(
                Config.PathFollowing.fixedLookahead,
                Config.PathFollowing.maxAccel,
                Config.PathFollowing.nominalDt,
                p,
                false,
                Config.PathFollowing.completionTolerance
        )
    }

    val pathFinished: Boolean
        get() = pathFollower?.isDone ?: true

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

        velocityHeadingPID.disable()
    }

    private fun convertToNative(d: DriveData) = DriveData(inchesPerSecondToTicks(d.left), inchesPerSecondToTicks(d.right))

    private fun updateVelocityHeading(): DriveData {
        println(deltaV)
        return DriveData(leftSetpoint - deltaV / 2, rightSetpoint + deltaV / 2)
    }

    private fun updatePathFollower(): DriveData {
        pathFollower?.let {
            val update = it.update(FramesOfReference.latestFieldToVehicle().value, Timer.getFPGATimestamp())
            val vel = Kinematics.inverseKinematics(update)
            val leftVel = vel.left
            val rightVel = vel.right

            leftSetpoint = inchesPerSecondToRPM(leftVel)
            rightSetpoint = inchesPerSecondToRPM(rightVel)
            gyroSetpoint = Rotation2D.fromRadians(update.deltaTheta)
        }
        return updateVelocityHeading()
    }

    private fun setMotors(controlMode: ControlMode, d: DriveData) {
        leftSide.set(controlMode, d.left)
        rightSide.set(controlMode, d.right)
    }

    private fun set(values: Pair<ControlMode, DriveData>) = setMotors(values.first, values.second)

    private fun invert() = DriveData(-leftSetpoint * if(inverted) -1 else 1, rightSetpoint * if(inverted) -1 else 1)

    override fun update() {
        set(when (Robot.CurrentMode) {
            RobotState.DISABLED -> {
                Pair(ControlMode.PercentOutput, DriveData(0.0, 0.0))
            }
            RobotState.TELEOP -> {
                Pair(ControlMode.PercentOutput, invert())
            }
            RobotState.AUTONOMOUS -> {
                if (!velocityHeadingPID.isEnabled) velocityHeadingPID.enable()
                Pair(
                        ControlMode.Velocity,
                        convertToNative(if (pathFollower != null)
                            updatePathFollower()
                        else if (useGyro)
                            updateVelocityHeading()
                        else DriveData(leftSetpoint, rightSetpoint))
                )
            }
        })
        networkTable.apply {
            putNumber("heading", gyroAngle.degrees)
            putNumber("left_setpoint", leftSetpoint)
            putNumber("right_setpoint", rightSetpoint)
            putNumber("left_velocity", leftVelocity)
            putNumber("right_velocity", rightVelocity)
            putNumber("left_position", leftPosition)
            putNumber("right_position", rightPosition)
            putNumber("left_error", leftSide.getClosedLoopError(0))
            putNumber("right_error", rightSide.getClosedLoopError(0))
            putString("current_mode", Robot.CurrentMode.name)
            FramesOfReference.latestFieldToVehicle().value.trans.apply {
                putNumber("x_coord", x)
                putNumber("y_coord", y)
            }
        }
        SmartDashboard.putNumber("left_velocity", leftVelocity)
        SmartDashboard.putNumber("left_position", leftPosition)
        SmartDashboard.putNumber("right_velocity", rightVelocity)
        SmartDashboard.putNumber("right_position", rightPosition)
        SmartDashboard.putNumber("heading", gyroAngle.degrees)
    }
}