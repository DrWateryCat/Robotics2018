package frc.team2186.robot.subsystems

import edu.wpi.first.wpilibj.Timer
import frc.team2186.robot.lib.interfaces.Subsystem
import frc.team2186.robot.lib.math.Rotation2D
import frc.team2186.robot.lib.odometry.FramesOfReference
import frc.team2186.robot.lib.odometry.Kinematics

object RobotPoseEstimator : Subsystem() {
    var leftEncoderPrev = 0.0
    var rightEncoderPrev = 0.0

    override fun update() {
        /*
        Drive.accessSync {
            val left = OldDrive.leftPosition
            val right = OldDrive.rightPosition

            val leftVel = OldDrive.leftVelocity
            val rightVel = OldDrive.rightVelocity

            val currentGyro = OldDrive.gyroAngle
            val time = Timer.getFPGATimestamp()

            val odometry = FramesOfReference.generateFromSensors(
                    left - leftEncoderPrev,
                    right - rightEncoderPrev,
                    currentGyro
            )

            val velocity = Kinematics.forwardKinematics(
                    leftVel,
                    rightVel
            )

            FramesOfReference.addObservations(time, odometry, velocity)

            leftEncoderPrev = left
            rightEncoderPrev = right
        }*/

        val left = Drive.leftPosition
        val right = Drive.rightPosition
        val leftVel = Drive.leftVelocity
        val rightVel = Drive.rightVelocity
        val currentGyro = Rotation2D.fromDegrees(Drive.heading)
        val time = Timer.getFPGATimestamp()
        val odometry = FramesOfReference.generateFromSensors(
                left - leftEncoderPrev,
                right - rightEncoderPrev,
                currentGyro
        )
        val velocity = Kinematics.forwardKinematics(
                leftVel,
                rightVel
        )

        FramesOfReference.addObservations(time, odometry, velocity)

        leftEncoderPrev = left
        rightEncoderPrev = right
    }
}