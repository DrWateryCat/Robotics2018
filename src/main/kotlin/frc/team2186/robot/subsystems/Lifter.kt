package frc.team2186.robot.subsystems

import com.google.gson.JsonObject
import edu.wpi.first.wpilibj.AnalogPotentiometer
import edu.wpi.first.wpilibj.PIDController
import edu.wpi.first.wpilibj.PIDOutput
import edu.wpi.first.wpilibj.Spark
import frc.team2186.robot.Config
import frc.team2186.robot.lib.interfaces.Subsystem

object Lifter : Subsystem() {
    private val motor = Spark(Config.Lifter.motorID)
    private val sensor = AnalogPotentiometer(Config.Lifter.sensorID, 360.0, 0.0)
    private val pidController = PIDController(Config.Lifter.kP, Config.Lifter.kI, Config.Lifter.kD, sensor, PIDOutput {
        set(it)
    }).apply {
        setInputRange(0.0, 360.0)
        setContinuous(false)
        setOutputRange(-1.0, 1.0)
        setPercentTolerance(0.9)
    }

    private var motorSetpoint = 0.0

    val done get() = pidController.onTarget()
    override val json get() = JsonObject().apply {
        addProperty("sensor_angle", sensor.get() * 360)
        addProperty("pid_setpoint", pidController.setpoint)
        addProperty("motor_setpoint", motorSetpoint)
        addProperty("using_pid", usePID)
    }
    var usePID = true

    @Synchronized
    fun set(input: Double) {
        if(usePID) pidController.setpoint = input else motorSetpoint = input
    }

    @Synchronized
    fun stop() {
        motorSetpoint = 0.0
        pidController.setpoint = 0.0
    }

    override fun update() {
        motor.set(motorSetpoint)
    }
}