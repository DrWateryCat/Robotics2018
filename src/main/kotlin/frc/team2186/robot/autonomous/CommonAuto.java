package frc.team2186.robot.autonomous;

import frc.team2186.robot.lib.common.IterativeAutoAction;
import frc.team2186.robot.lib.math.Rotation2D;
import frc.team2186.robot.subsystems.Drive;
import kotlin.Unit;
import kotlin.jvm.functions.Function0;

public class CommonAuto {
    public static IterativeAutoAction goDistance(double distance, double speedInIPS) {
        return new IterativeAutoAction(() -> {
            Drive.INSTANCE.setForwardVelocity(speedInIPS);
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

    public static IterativeAutoAction baseline(double speed) {
        return new IterativeAutoAction(() -> {
            Drive.INSTANCE.setForwardVelocity(speed);
            return Drive.INSTANCE.getLeftPosition() > 103;
        });
    }
    public static IterativeAutoAction halfBaseline(double speed){
        return new IterativeAutoAction(() -> {
            Drive.INSTANCE.setForwardVelocity(speed);
            return Drive.INSTANCE.getLeftPosition() > 103 / 2;
        });
    }
}
