package frc.team2186.robot.lib.odometry

import frc.team2186.robot.lib.math.RigidTransform2D
import frc.team2186.robot.lib.math.Rotation2D
import kotlin.math.abs

object Kinematics {
    var trackScrubFactor = 0.5
    var wheelDiameter = 6.0
    var effectiveWheelDiameter = 6.0

    fun forwardKinematics(leftWheelDelta: Double, rightWheelDelta: Double): RigidTransform2D.Delta {
        val linearVel = (leftWheelDelta + rightWheelDelta) / 2
        val deltaV = (rightWheelDelta - leftWheelDelta) / 2
        val deltaTheta = deltaV * 2 * trackScrubFactor / effectiveWheelDiameter

        return RigidTransform2D.Delta(
                linearVel,
                0.0,
                deltaTheta
        )
    }

    fun forwardKinematics(leftWheelDelta: Double,
                          rightWheelDelta: Double,
                          deltaRotation: Double) = RigidTransform2D.Delta((leftWheelDelta + rightWheelDelta) / 2,
                                                                          0.0,
                                                                                 deltaRotation)

    fun integrateForwardKinematics(currentPose: RigidTransform2D,
                          leftWheelDelta: Double,
                          rightWheelDelta: Double,
                          currentHeading: Rotation2D): RigidTransform2D {
        val withGyro = forwardKinematics(leftWheelDelta, rightWheelDelta, currentPose.rot.inverse().rotateBy(currentHeading).radians)
        return currentPose.transformBy(RigidTransform2D.fromVelocity(withGyro))
    }

    class DriveVelocity(val left:Double, val right:Double)

    fun inverseKinematics(velocity: RigidTransform2D.Delta): DriveVelocity {
        if (abs(velocity.deltaTheta) < 1E-9) {
            return DriveVelocity(velocity.deltaX, velocity.deltaY)
        }

        val deltaV = wheelDiameter * velocity.deltaTheta / (2 * trackScrubFactor)
        return DriveVelocity(velocity.deltaX - deltaV, velocity.deltaX + deltaV)
    }
}