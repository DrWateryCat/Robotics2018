package frc.team2186.robot.autonomous;

import frc.team2186.robot.lib.common.ActionRunner;
import frc.team2186.robot.lib.interfaces.SequentialAutonomousMode;
import frc.team2186.robot.subsystems.Drive;
import kotlin.Unit;
import org.jetbrains.annotations.NotNull;

import static frc.team2186.robot.autonomous.CommonAuto.*;

public class BaselineJava extends SequentialAutonomousMode {
    public BaselineJava() {
        super("Baseline in Java", false);
    }

    @Override
    @NotNull
    public ActionRunner getActions() {
        ActionRunner ar = new ActionRunner();
        ar.action(goDistance(120, 10));
        ar.actionComplete(stop());

        return ar;
    }
}
