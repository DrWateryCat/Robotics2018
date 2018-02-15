package frc.team2186.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.google.gson.JsonObject
import edu.wpi.first.wpilibj.*
import frc.team2186.robot.Config
import frc.team2186.robot.lib.common.*
import frc.team2186.robot.lib.interfaces.Subsystem

object Lifter : Subsystem() {
    private val elevator = CANTalon(Config.Lifter.masterID).apply {
        configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0)
    } + CANVictor(Config.Lifter.slaveID).apply {

    }
    private val bar = VictorSP(Config.Lifter.barMotorID)

    private val barDownSensor = DigitalInput(Config.Lifter.barDownLimitSwitchID)
    private val barUpSensor = DigitalInput(Config.Lifter.barUpLimitSwitchID)
    private val elevatorFullDownSensor = DigitalInput(Config.Lifter.elevatorFullLow)
    private val elevatorFullHighSensor = DigitalInput(Config.Lifter.elevatorFullHigh)

    private var elevatorSetpoint = 0.0
    private var barSetpoint = 0.0

    val pidController = PIDController(Config.Lifter.kP, Config.Lifter.kI, Config.Lifter.kD, PIDInput {
        elevator.getSelectedSensorPosition(0).toDouble()
    }) {
        accessSync {
            elevatorSetpoint = it
        }
    }.apply {
        setInputRange(0.0, 1.0)
        setOutputRange(-0.5, 0.5)
        setContinuous(false)
        setPercentTolerance(0.95)
    }

    fun feetToPercent(feet: Double) = feet / 7.0
    fun inchesToPercent(inches: Double) = inches / 84.0

    @get:Synchronized
    val done
        get() = pidController.onTarget()

    @get:Synchronized
    val isFullUp
        get() = elevatorFullHighSensor.get().not()

    @get:Synchronized
    val isFullDown
        get() = elevatorFullDownSensor.get().not()

    @Synchronized
    fun fullUp() {
        set(if (isFullUp) 0.0 else (if (usePID) 1.0 else 0.5))
    }

    @Synchronized
    fun fullDown() {
        set(0.0)
    }

    @get:Synchronized
    override val json
        get() = JsonObject().apply {
        addProperty("motor_setpoint", elevatorSetpoint)
        addProperty("using_pid", usePID)
    }

    @set:Synchronized
    var usePID = true

    @Synchronized
    fun set(input: Double) {
        if(usePID) pidController.setpoint = input else elevatorSetpoint = input
    }

    @Synchronized
    fun stop() {
        elevatorSetpoint = 0.0
        pidController.setpoint = 0.0
        usePID = false
    }

    @Synchronized
    fun barUp() {
        barSetpoint = if (isBarUp) 0.0 else 0.25
    }

    @Synchronized
    fun barDown() {
        barSetpoint = if (isBarDown) 0.0 else 0.25
    }

    @get:Synchronized
    val isBarDown: Boolean
        get() = barDownSensor.get().not()

    @get:Synchronized
    val isBarUp: Boolean
        get() = barUpSensor.get().not()

    override fun update() {
        elevator.set(ControlMode.PercentOutput, elevatorSetpoint)
        bar.set(barSetpoint)
    }
}