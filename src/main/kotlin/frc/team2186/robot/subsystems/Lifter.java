package frc.team2186.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.google.gson.JsonObject;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.VictorSP;
import frc.team2186.robot.Config;
import frc.team2186.robot.lib.interfaces.Subsystem;
import org.jetbrains.annotations.NotNull;

public class Lifter extends Subsystem {
    private WPI_TalonSRX scissorLiftMotor;
    private VictorSP flipper;

    private DigitalInput elevatorFullDown;
    private DigitalInput elevatorFullUp;
    private DigitalInput barFullDown;
    private DigitalInput barFullUp;

    private double scissorSetpoint = 0.0;
    private double flipperSetpoint = 0.0;

    public static Lifter INSTANCE = new Lifter();
    private Lifter() {
        super();
        scissorLiftMotor = new WPI_TalonSRX(Config.Lifter.masterID);
        flipper = new VictorSP(Config.Lifter.barMotorID);

        elevatorFullDown = new DigitalInput(Config.Lifter.elevatorFullLow);
        elevatorFullUp = new DigitalInput(Config.Lifter.elevatorFullHigh);
        barFullDown = new DigitalInput(Config.Lifter.barDownLimitSwitchID);
        barFullUp = new DigitalInput(Config.Lifter.barUpLimitSwitchID);
    }

    public boolean isElevatorDown() {
        return elevatorFullDown.get();
    }

    public boolean isElevatorUp() {
        return elevatorFullUp.get();
    }

    public boolean isBarFullUp() {
        return barFullUp.get();
    }

    public boolean isBarFullDown() {
        return barFullDown.get();
    }

    public void set(double input) {
        this.scissorSetpoint = input;
    }

    @NotNull
    @Override
    public JsonObject getJson() {
        return null;
    }

    @Override
    public void update() {
        if ((!isElevatorDown() || this.scissorSetpoint >= 0) && (!isElevatorUp() || this.scissorSetpoint <= 0)) {
            this.scissorLiftMotor.set(ControlMode.PercentOutput, this.scissorSetpoint);
        } else {
            this.scissorLiftMotor.set(ControlMode.PercentOutput, 0);
        }

        if ((!isBarFullDown() || this.flipperSetpoint >= 0) && (!isBarFullUp() || this.scissorSetpoint <= 0)) {
            this.flipper.set(this.flipperSetpoint);
        } else {
            this.flipper.set(0);
        }
    }
}
