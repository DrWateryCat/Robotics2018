package frc.team2186.robot.autonomous;

import frc.team2186.robot.lib.interfaces.AutonomousMode;

public class DoNothing extends AutonomousMode {
    public DoNothing() {
        super("Do Nothing", true);
    }

    @Override
    public void update() {
    }

    @Override
    public void init() {
    }

    @Override
    public boolean done() {
        return true;
    }
}
