package frc.team2186.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.Timer
import frc.team2186.robot.Config
import frc.team2186.robot.common.SynchronousPID
import frc.team2186.robot.lib.common.CANTalon
import frc.team2186.robot.lib.common.CANVictor
import frc.team2186.robot.lib.common.plus
import frc.team2186.robot.lib.interfaces.Subsystem
import frc.team2186.robot.lib.math.Rotation2D
import frc.team2186.robot.lib.networking.EasyNetworkTable
import frc.team2186.robot.lib.odometry.FramesOfReference
import frc.team2186.robot.lib.odometry.Kinematics
import frc.team2186.robot.lib.pathfinding.Path
import frc.team2186.robot.lib.pathfinding.PurePursuitController
import kotlin.math.abs
import kotlin.math.max

/*
* This is a reimplementation of the drive subsystem
* In theory this can be a drop-in replacement for the old drive
* while also being easier to read and (hopefully) better at what it does
*/

object Drive : Subsystem() {
    /*
    * Create all the objects needed
    * All these should be private
    * The theory is we should only expose
    * what we want to expose to the rest
    * of the API
    */
    private enum class State {
        TELEOP, VELOCITY_SETPOINT, VELOCITY_HEADING, PATH_FOLLOWING
    }

    //Left Motors
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

    //Right Motors
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

    //NavX
    private val navx = AHRS(SPI.Port.kMXP)

    //Current State
    private var currentState = State.TELEOP

    //Network Table
    private val table = EasyNetworkTable("/drive")

    //Path follower
    //This is null by default
    //Then is given a value when told to follow a path
    private var pathFollower: PurePursuitController? = null

    //Synchronous PID
    //It's a PID controller that runs in the same thread as drive
    private val headingController = SynchronousPID(
            Config.Drive.kHeadingP,
            Config.Drive.kHeadingI,
            Config.Drive.kHeadingD
    )

    //Left setpoint
    //In teleop this will be a value [-1.0, 1.0]
    //Else this will be RPM
    private var leftSetpoint = 0.0

    //Right setpoint
    //In teleop this will be a value [-1.0, 1.0]
    //Else this will be RPM
    private var rightSetpoint = 0.0

    //Gyro setpoint
    //This should only be used in velocity heading mode
    //It is a value from [0.0, 360.0)
    private var gyroSetpoint = 0.0

    /*
     * Math Functions
     * These are needed to convert
     * between all the different
     * units everything likes to use
    */

    fun ticksToInches(ticks: Double) = ticks / Config.Drive.ticksPerRevolution * Config.Drive.wheelCircumference
    fun inchesToTicks(inches: Double) = (inches / Config.Drive.wheelCircumference) * Config.Drive.ticksPerRevolution
    fun rpmToTicks(rpm: Double) = rpm * Config.Drive.ticksPer100ms
    fun ticksToRPM(ticks: Double) = ticks / Config.Drive.ticksPer100ms
    fun rpmToInchesPerSecond(rpm: Double) = rpm * Config.Drive.wheelCircumference / 60
    fun inchesPerSecondToRPM(ips: Double) = ips * 60 / Config.Drive.wheelCircumference
    fun ticksToInchesPerSecond(ticks: Double) = rpmToInchesPerSecond(ticksToRPM(ticks))
    fun inchesPerSecondToTicks(ips: Double) = rpmToTicks(inchesPerSecondToRPM(ips))

    /*
    * Public values come next
    * All of these should be immutable
    * And are just getters for the value.
    */

    val leftPosition: Double
        get() = ticksToInches(-leftSide.getSelectedSensorPosition(0).toDouble())
    val rightPosition: Double
        get() = ticksToInches(rightSide.getSelectedSensorPosition(0).toDouble())
    val leftSpeed: Double
        get() = ticksToRPM(-leftSide.getSelectedSensorVelocity(0).toDouble())
    val rightSpeed: Double
        get() = ticksToRPM(rightSide.getSelectedSensorVelocity(0).toDouble())
    val leftVelocity: Double
        get() = ticksToInchesPerSecond(-leftSide.getSelectedSensorVelocity(0).toDouble())
    val rightVelocity: Double
        get() = ticksToInchesPerSecond(rightSide.getSelectedSensorVelocity(0).toDouble())
    val heading: Double
        get() = navx.yaw.toDouble()
    val finishedPath: Boolean
        get() = pathFollower?.isDone ?: true

    /*
    * Public methods
    * These are what are actually
    * Called by outside sources
    * Like autonomous modes
    */

    fun tankDrive(left: Double, right: Double) {
        if (currentState != State.TELEOP) {
            currentState = State.TELEOP
        }

        leftSetpoint = left
        rightSetpoint = right
    }

    fun setForwardVelocity(ips: Double) {
        if (currentState != State.VELOCITY_SETPOINT) {
            currentState = State.VELOCITY_SETPOINT
        }
        leftSetpoint = inchesPerSecondToRPM(ips)
        rightSetpoint = inchesPerSecondToRPM(ips)
    }

    fun setVelocityVector(vel: Double, heading: Double) {
        if (currentState != State.VELOCITY_HEADING) {
            currentState = State.VELOCITY_HEADING
        }

        leftSetpoint = vel
        rightSetpoint = vel
        gyroSetpoint = heading
    }

    fun followPath(p: Path) {
        if (currentState != State.PATH_FOLLOWING) {
            currentState = State.PATH_FOLLOWING
        }
        pathFollower = PurePursuitController(
                Config.PathFollowing.fixedLookahead,
                Config.PathFollowing.maxAccel,
                Config.PathFollowing.nominalDt,
                p,
                false,
                Config.PathFollowing.completionTolerance
        )
    }

    //Reset the encoders and gyro
    fun reset() {
        leftSide.setSelectedSensorPosition(0, 0, 0)
        rightSide.setSelectedSensorPosition(0, 0, 0)
        navx.reset()
    }

    /*
    * Private methods
    * These are used by update()
    * To handle the velocity setpoints
    * and path follower
    *
    * NOTE: We do not need an updater for teleop or velocity setpoint
    * The talons handle that.
    */

    private fun updateVelocityHeading() {
        //First determine the heading error
        //I'm using my rotation2d math lib for this
        //But in theory it's just (theta setpoint) - (actual theta)
        val actualTheta = Rotation2D.fromDegrees(heading)
        val headingError = Rotation2D.fromDegrees(gyroSetpoint).rotateBy(actualTheta.inverse()).degrees

        //Next get the delta between the 2 sides
        //Computed from the PID controller
        val delta = headingController.calculate(headingError)

        //Now compute the wheel velocities and set
        val left = leftSetpoint + delta / 2
        val right = rightSetpoint + delta / 2

        setVelocity(left, right)
    }

    private fun updatePathFollower() {
        //Assert that the path follower is not null
        //If it is then there's something big wrong.
        pathFollower!!.let {
            //First get the current robot pose
            val currentPose = FramesOfReference.latestFieldToVehicle().value

            //Next get the velocity command for the current path
            val command = it.update(currentPose, Timer.getFPGATimestamp())

            //Next use kinematics to get the desired left and right velocities
            var (left, right) = Kinematics.inverseKinematics(command)

            //Now scale the velocities down based on the max velocity
            var maxVel = 0.0
            maxVel = max(maxVel, abs(left))
            maxVel = max(maxVel, abs(right))

            if (maxVel >= Config.PathFollowing.maxVelocity) {
                val scaling = Config.PathFollowing.maxVelocity / maxVel
                left *= scaling
                right *= scaling
            }

            //Now set
            setVelocity(left, right)
        }
    }

    //I don't want to rewrite code that I don't have to
    private fun setVelocity(left: Double, right: Double) {
        leftSide.set(ControlMode.Velocity, left)
        rightSide.set(ControlMode.Velocity, right)
    }

    //Update networktables
    private fun updateNT() {
        table["left_position"] = leftPosition
        table["right_position"] = rightPosition
        table["heading"] = heading
        table["left_velocity"] = leftVelocity
        table["right_velocity"] = rightVelocity
        FramesOfReference.latestFieldToVehicle().value.apply {
            table["x_coord"] = trans.x
            table["y_coord"] = trans.y
        }
    }

    override fun update() {
        when(currentState) {
            State.TELEOP -> {
                leftSide.set(ControlMode.PercentOutput, leftSetpoint)
                rightSide.set(ControlMode.PercentOutput, -rightSetpoint)
            }
            State.VELOCITY_SETPOINT -> {
                setVelocity(leftSetpoint, rightSetpoint)
            }
            State.VELOCITY_HEADING -> {
                updateVelocityHeading()
            }
            State.PATH_FOLLOWING -> {
                updatePathFollower()
            }
        }
        updateNT()
    }
}