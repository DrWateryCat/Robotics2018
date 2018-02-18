package frc.team2186.robot.autonomous;

import frc.team2186.robot.lib.common.ActionRunner;
import frc.team2186.robot.lib.interfaces.SequentialAutonomousMode;
import org.jetbrains.annotations.NotNull;

import static frc.team2186.robot.autonomous.CommonAuto.baseline;
import static frc.team2186.robot.autonomous.CommonAuto.stop;

public class BaselineJava extends SequentialAutonomousMode {
    private ActionRunner ar = new ActionRunner();
    public BaselineJava() {
        super("Baseline in Java", false);
        ar.action(baseline(false));
        ar.actionComplete(stop());
    }

    @Override
    @NotNull
    public ActionRunner getActions() {
        return ar;
    }
}
