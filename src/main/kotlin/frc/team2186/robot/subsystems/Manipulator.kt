package frc.team2186.robot.subsystems

import com.google.gson.JsonObject
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.interfaces.Potentiometer
import frc.team2186.robot.Config
import frc.team2186.robot.lib.interfaces.Subsystem

object Manipulator : Subsystem(), PIDOutput {
    val sensor = AnalogPotentiometer(Config.Manipulator.sensorID, 2880.0, 0.0)
    val motor = Spark(Config.Manipulator.motorID)

    var setpoint: Double set(value) {
        positionController.setpoint = value
    } get() {
        return positionController.setpoint
    }

    val currentPosition: Double = sensor.get() / 2880.0

    override val json: JsonObject
        get() = JsonObject().apply {
        }

    private var motorVal = 0.0

    private val positionController = PIDController(
            Config.Manipulator.kP,
            Config.Manipulator.kI,
            Config.Manipulator.kD,
            sensor,
            this
    )

    init {
        positionController.apply {
            setOutputRange(-1.0, 1.0)
            setInputRange(0.0, 2880.0)
        }
    }

    @Synchronized
    fun stop() {
        positionController.disable()
        motor.set(0.0)
    }

    fun enablePID() = positionController.enable()
    fun disablePID() = positionController.disable()

    override fun update() {
        motor.set(motorVal)
    }

    override fun pidWrite(output: Double) {
        motorVal = output
    }
}