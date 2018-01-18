package frc.team2186.robot

object Config {
    object Drive {
        const val leftMasterID = 0
        const val leftSlaveID = 1
        const val rightMasterID = 2
        const val rightSlaveID = 3

        const val trackScrubFactor: Double = 0.5
        const val wheelDiameter: Double = 6.0

        const val ticksPerRevolution = 1440.0
        const val ticksPer100ms = ticksPerRevolution / 600

        const val kLeftP = 1.0
        const val kLeftI = 0.0
        const val kLeftD = 0.0
        const val kLeftF = 0.05

        const val kRightP = 1.0
        const val kRightI = 0.0
        const val kRightD = 0.0
        const val kRightF = 0.05
    }

    object Manipulator {
        const val motorID = 0
        const val sensorID = 0
        const val kP = 0.0
        const val kI = 0.0
        const val kD = 0.0
    }

    object PathFollowing {
        const val fixedLookahead = 24.0
        const val maxAccel = 100.0
        const val nominalDt = 0.0
        const val completionTolerance = 0.25
        const val maxVelocity = 100.0
    }

    object Controls {
        const val leftJoystickID = 0
        const val rightJoystickID = 1
    }
}