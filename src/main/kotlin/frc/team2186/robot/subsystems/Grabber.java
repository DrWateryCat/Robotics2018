package frc.team2186.robot.subsystems;

import frc.team2186.robot.Config;
import frc.team2186.robot.common.Motor;
import frc.team2186.robot.lib.interfaces.Subsystem;
import frc.team2186.robot.lib.networking.EasyNetworkTable;

public class Grabber extends Subsystem {
    private static Grabber INSTANCE = new Grabber();
    public static Grabber getInstance() {
        return INSTANCE;
    }

    private Motor left = new Motor(Config.Grabber.Left.INSTANCE);
    private Motor right = new Motor(Config.Grabber.Right.INSTANCE);

    private EasyNetworkTable networkTable = new EasyNetworkTable("/grabber");

    private Grabber() {
        super();
        right.setInverted(true);
    }

    private double setpoint = 0.0;

    public double getSetpoint() {
        return setpoint;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public boolean hasBox() {
        return left.isStalled() && right.isStalled();
    }

    @Override
    public void update() {
        left.set(setpoint);
        right.set(setpoint);
    }
}
