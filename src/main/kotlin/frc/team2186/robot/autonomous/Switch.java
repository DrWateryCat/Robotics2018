package frc.team2186.robot.autonomous;

import frc.team2186.robot.Robot;
import frc.team2186.robot.common.SwitchState;
import frc.team2186.robot.lib.common.ActionRunner;
import frc.team2186.robot.lib.interfaces.SequentialAutonomousMode;
import org.jetbrains.annotations.NotNull;

import static frc.team2186.robot.autonomous.CommonAuto.*;

public class Switch extends SequentialAutonomousMode {
    private ActionRunner ar = new ActionRunner();
    public Switch() {
        super("Switch", false);
        switch (Robot.Companion.getStartingPosition()) {
            case LEFT: {
                if (Robot.Companion.getStartingSwitch() == SwitchState.LEFT) {
                    ar.action(baseline(false));
                    ar.action(dropBox());
                } else {
                    ar.action(moveOne());
                    ar.action(heading90());
                    ar.action(crossBoard(false));
                    ar.action(heading270());
                    ar.action(moveTwo());
                    ar.action(dropBox());
                }
            }
            break;
            case MIDDLE: {
                if(Robot.Companion.getStartingSwitch() == SwitchState.LEFT) {
                    ar.action(moveOne());
                    ar.action(heading270());
                    ar.action(crossBoard(true));
                    ar.action(heading90());
                    ar.action(moveTwo());
                    ar.action(dropBox());
                } else {
                    ar.action(moveOne());
                    ar.action(heading90());
                    ar.action(crossBoard(true));
                    ar.action(moveTwo());
                    ar.action(dropBox());
                }
            }
            break;
            case RIGHT: {
                if(Robot.Companion.getStartingSwitch() == SwitchState.LEFT) {
                    ar.action(moveOne());
                    ar.action(heading270());
                    ar.action(crossBoard(false));
                    ar.action(heading90());
                    ar.action(moveTwo());
                    ar.action(dropBox());
                } else {
                    ar.action(baseline(false));
                    ar.action(dropBox());
                }
            }
            break;
        }
    }

    @NotNull
    @Override
    public ActionRunner getActions() {
        return ar;
    }
}
