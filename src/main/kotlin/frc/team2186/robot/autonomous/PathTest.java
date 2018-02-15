package frc.team2186.robot.autonomous;

import frc.team2186.robot.lib.common.ActionRunner;
import frc.team2186.robot.lib.interfaces.SequentialAutonomousMode;
import frc.team2186.robot.lib.pathfinding.Path;
import frc.team2186.robot.subsystems.Drive;
import org.jetbrains.annotations.NotNull;

import static frc.team2186.robot.autonomous.CommonAuto.stop;

public class PathTest extends SequentialAutonomousMode {
    private Path path;
    public PathTest() {
        super("Path in java", false);

        path = new Path(
                new Path.Waypoint(path.translation(0, 0), 0, "")
        );
    }

    @NotNull
    @Override
    public ActionRunner getActions() {
        ActionRunner ret = new ActionRunner();
        ret.action(() -> {
            Drive.INSTANCE.followPath(path, false);

            return Drive.INSTANCE.getFinishedPath();
        });

        ret.actionComplete(stop());

        return ret;
    }
}
