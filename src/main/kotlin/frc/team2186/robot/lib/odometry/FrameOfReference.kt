package frc.team2186.robot.lib.odometry

import frc.team2186.robot.lib.math.InterpolatingDouble
import frc.team2186.robot.lib.math.InterpolatingTreeMap
import frc.team2186.robot.lib.math.RigidTransform2D

class FrameOfReference(bufferSize: Int = 100){
    private val buffer = InterpolatingTreeMap<InterpolatingDouble, RigidTransform2D>(bufferSize)
    private var delta = RigidTransform2D.Delta(0.0, 0.0, 0.0)

    init {
        reset(0.0, RigidTransform2D())
    }

    fun reset(start: Double, initial: RigidTransform2D) {
        buffer.clear()
        buffer[InterpolatingDouble(start)] = initial
    }

    fun current(timestamp: Double) = buffer.getInterpolated(InterpolatingDouble(timestamp)) ?: RigidTransform2D.identity
    fun latest() = buffer.lastEntry()
    fun predicted(lookahead: Double) = latest().value.transformBy(RigidTransform2D.fromVelocity(
            delta.scale(lookahead)
    ))

    fun observe(timestamp: Double, observation: RigidTransform2D, delta: RigidTransform2D.Delta) {
        buffer[InterpolatingDouble(timestamp)] = observation
        this.delta = delta
    }
}