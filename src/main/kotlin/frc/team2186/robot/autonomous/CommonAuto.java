package frc.team2186.robot.autonomous;

import frc.team2186.robot.Config;
import frc.team2186.robot.lib.common.IterativeAutoAction;
import frc.team2186.robot.lib.math.Rotation2D;
import frc.team2186.robot.subsystems.Drive;
import frc.team2186.robot.subsystems.Platform;
import kotlin.Unit;
import kotlin.jvm.functions.Function0;

public class CommonAuto {
    public static IterativeAutoAction goDistance(double distance) {
        return new IterativeAutoAction(() -> {
            Drive.INSTANCE.setForwardVelocity(Config.Auto.speed);
            return Drive.INSTANCE.getLeftPosition() > distance;
        });
    }

    public static IterativeAutoAction turnToAngle(double angle) {
        return new IterativeAutoAction(() -> {
            Drive.INSTANCE.setGyroSetpoint(Rotation2D.Companion.fromDegrees(angle));

            return Drive.INSTANCE.getGyroAngle().getDegrees() == angle;
        });
    }

    public static Function0<Unit> stop() {
        return () -> {
            Drive.INSTANCE.stop();
            return Unit.INSTANCE;
        };
    }

    public static IterativeAutoAction baseline(boolean half) {
        return half ? goDistance(103 / 2) : goDistance(103);
    }

    public static IterativeAutoAction crossBoard(boolean half) {
        return half ? goDistance(137.5 / 2) : goDistance(137.5);
    }

    public static IterativeAutoAction heading90() {
        return turnToAngle(90);
    }

    public static IterativeAutoAction heading270() {
        return turnToAngle(270);
    }

    public static IterativeAutoAction dropBox() {
        return new IterativeAutoAction(() -> {
            Platform.INSTANCE.setSetpoint(0.25);

            if (Platform.INSTANCE.isBarUp()) {
                Platform.INSTANCE.setSetpoint(-0.25);
                return Platform.INSTANCE.isBarDown();
            }
            return false;
        });
    }

    public static IterativeAutoAction moveOne() {
        return goDistance(80);
    }

    public static IterativeAutoAction moveTwo() {
        return goDistance(59);
    }
}
