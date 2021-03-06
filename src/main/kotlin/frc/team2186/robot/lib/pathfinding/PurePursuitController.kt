package frc.team2186.robot.lib.pathfinding

import frc.team2186.robot.lib.math.RigidTransform2D
import frc.team2186.robot.lib.math.Rotation2D
import frc.team2186.robot.lib.math.Translation2D
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.sign
import kotlin.math.sqrt

class PurePursuitController (
        val fixedLookahead: Double,
        val maxAccel: Double,
        val nominalDt: Double,
        var path: Path,
        val reversed: Boolean,
        val completionTolerance: Double
) {
    data class Circle(val center: Translation2D, val radius: Double, val turnRight: Boolean)

    private var lastCommand: RigidTransform2D.Delta? = null
    var lastTime: Double = 0.0
    var deltaTime: Double = nominalDt

    val isDone: Boolean
        get() = path.remainingDistance() <= completionTolerance

    fun update(currentPose: RigidTransform2D, now: Double): RigidTransform2D.Delta {
        var pose = currentPose

        if (reversed) {
            pose = RigidTransform2D(currentPose.trans, currentPose.rot.rotateBy(Rotation2D.fromRadians(PI)))
        }

        val distanceFromPath = path.update(pose.trans)

        if (isDone) {
            return RigidTransform2D.Delta(0.0, 0.0, 0.0)
        }

        val lookaheadPoint = path.getLookaheadPoint(pose.trans, distanceFromPath + fixedLookahead)
        val circle = joinPath(pose, lookaheadPoint.translation)

        var speed = lookaheadPoint.speed * (if (reversed) -1 else 1)

        val deltaT = now - lastTime

        if (lastCommand == null) {
            lastCommand = RigidTransform2D.Delta(0.0, 0.0, 0.0)
            deltaTime = deltaT
        }

        val accel = (speed - lastCommand!!.deltaX) / deltaT
        if (accel < -maxAccel) {
            speed = lastCommand!!.deltaX - maxAccel * deltaT
        } else if (accel > maxAccel) {
            speed = lastCommand!!.deltaX + maxAccel * deltaT
        }

        val remainingDistance = path.remainingDistance()
        val maxAllowedSpeed = sqrt(2 * maxAccel * remainingDistance)

        if (abs(speed) > maxAllowedSpeed) {
            speed = maxAllowedSpeed * sign(speed)
        }

        val minSpeed = 4.0
        if (abs(speed) < 4.0) {
            speed = minSpeed * sign(speed)
        }

        val ret = if (circle != null) {
            RigidTransform2D.Delta(
                    speed,
                    0.0,
                    (if (circle.turnRight) -1 else 1 ) * abs(speed) / circle.radius
            )
        } else {
            RigidTransform2D.Delta(speed, 0.0, 0.0)
        }

        lastTime = now
        lastCommand = ret

        return ret
    }

    companion object {
        fun joinPath(pose: RigidTransform2D, lookaheadPoint: Translation2D): Circle? {
            val x1 = pose.trans.x
            val y1 = pose.trans.y

            val x2 = lookaheadPoint.x
            val y2 = lookaheadPoint.y

            val poseToLookahead = pose.trans.inverse().translateBy(lookaheadPoint)

            val crossProduct = poseToLookahead.x * pose.rot.sin - pose.trans.y * pose.rot.cos

            if (crossProduct < 1E-9) {
                return null
            }

            val deltaX = x2 - x1
            val deltaY = y2 - y1

            val my = (if (crossProduct > 0) -1 else 1) * pose.rot.cos
            val mx = (if (crossProduct > 0) 1 else -1) * pose.rot.sin

            val crossTerm = mx * deltaX + my * deltaY

            if (abs(crossTerm) < 1E-9) {
                //Colinear
                return null
            }

            return Circle(
                    Translation2D(
                            (mx * (x1 * x1 - x2 * x2 - deltaY * deltaY) + 2 * my * x1 * deltaY) / (2 * crossTerm),
                            (-my * (-y1 * y1 + y2 * y2 + deltaX * deltaX) + 2 * mx * y1 * deltaX) / (2 * crossTerm)
                    ),
                    0.5 * abs((deltaX * deltaX + deltaY * deltaY) / crossTerm),
                    crossProduct > 0
            )
        }
    }

    fun markersCrossed(): Set<String> = path.markersCrossed
}