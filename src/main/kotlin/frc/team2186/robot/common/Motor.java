package frc.team2186.robot.common;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.VictorSP;
import frc.team2186.robot.Config;
import frc.team2186.robot.Robot;

public class Motor implements SpeedController {
    private VictorSP motor;
    private PowerDistributionPanel pdp;

    private int pdpChannel = 0;
    private double peakCurrent = 0.0;

    private double setpoint = 0.0;
    public Motor(Config.Grabber.Motor cfg) {
        this.motor = new VictorSP(cfg.getId());
        this.pdp = Robot.Companion.getPdp();
        this.pdpChannel = cfg.getPdpChannel();
        this.peakCurrent = cfg.getPeakCurrent();
    }

    @Override
    public void set(double speed) {
        setpoint = speed;
        motor.set(setpoint);
    }

    @Override
    public double get() {
        return setpoint;
    }

    @Override
    public void setInverted(boolean isInverted) {
        motor.setInverted(isInverted);
    }

    @Override
    public boolean getInverted() {
        return motor.getInverted();
    }

    @Override
    public void disable() {
        motor.disable();
    }

    @Override
    public void stopMotor() {
        motor.stopMotor();
    }

    @Override
    public void pidWrite(double output) {
        setpoint = output;
        motor.pidWrite(output);
    }

    public double getCurrent() {
        return pdp.getCurrent(pdpChannel);
    }

    public boolean isStalled() {
        return getCurrent() >= peakCurrent;
    }
}
