package frc.team2186.robot

object Config {
    object Drive {
        const val leftMasterID = 0
        const val leftSlaveID = 1
        const val rightMasterID = 2
        const val rightSlaveID = 3

        const val wheelDiameter: Double = 6.0
        const val wheelBaseLength = 23.5
        const val trackScrubFactor: Double = 0.5
        const val effectiveWheelDiameter = (wheelDiameter * wheelDiameter + wheelBaseLength * wheelBaseLength) / wheelBaseLength

        const val ticksPerRevolution = 1440.0
        const val ticksPer100ms = ticksPerRevolution / 600

        const val kLeftP = 0.52
        const val kLeftI = 0.0
        const val kLeftD = 0.425
        const val kLeftF = 0.3

        const val kRightP = 0.52
        const val kRightI = 0.0
        const val kRightD = 0.425
        const val kRightF = 0.3

        const val kHeadingP = 0.1
        const val kHeadingI = 0.0
        const val kHeadingD = 0.0
        const val kHeadingF = 0.0
    }

    object Platform {
        const val lifterMaster = 3
        const val lifterSlave = 4
        const val motorID = 1
        const val barDownSwitch = 1
        const val barUpSwitch = 0
    }

    object Grabber {
        abstract class Motor {
            abstract val id: Int
            abstract val pdpChannel: Int
            abstract val peakCurrent: Double
        }
        object Left : Motor() {
            override val id = 0
            override val pdpChannel = 0
            override val peakCurrent = 30.0
        }

        object Right : Motor() {
            override val id = 5
            override val pdpChannel = 1
            override val peakCurrent = 30.0
        }
    }

    object PathFollowing {
        const val fixedLookahead = 24.0
        const val maxAccel = 100.0
        const val nominalDt = 20.0
        const val completionTolerance = 0.25
        const val maxVelocity = 100.0

        val waypointsFile: String = javaClass.classLoader.getResource("waypoints.json")!!.file
    }

    object Controls {
        const val leftJoystickID = 0
        const val rightJoystickID = 1
        const val codriverJoystickID = 2

        const val lifterUpButton = 3
        const val grabberButton = 1

        const val invertButton = 11
    }

    object Camera {
        const val cameraID = 0
        const val cameraPort = 5801
    }

    object Auto {
        const val speed = 80.0
    }
}