package frc.team2186.robot.lib.odometry

import edu.wpi.first.wpilibj.Timer
import frc.team2186.robot.subsystems.Drive
import java.util.logging.Logger

object RobotPoseEstimator {
    var leftEncoderPrev = 0.0
    var rightEncoderPrev = 0.0

    fun run() {
        Logger.getLogger("RobotPoseEstimator").info("Starting Pose Estimation thread")
        while(Thread.interrupted().not()) {
            val left = Drive.leftPosition
            val right = Drive.rightPosition

            val time = Timer.getFPGATimestamp()

            val leftVel = Drive.leftVelocity
            val rightVel = Drive.rightVelocity

            val currentGyro = Drive.gyroAngle

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
}