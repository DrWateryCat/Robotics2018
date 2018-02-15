package frc.team2186.robot.autonomous;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team2186.robot.lib.interfaces.AutonomousMode;
import frc.team2186.robot.subsystems.Drive;

import java.io.File;
import java.io.FileInputStream;
import java.io.InputStreamReader;

public class PlayAuto extends AutonomousMode {
    private File autoData;
    private FileInputStream is;
    private JsonElement json;
    private boolean canRun = true;
    public PlayAuto() {
        super("Play a recorded auto", false);
    }

    @Override
    public boolean done() {
        return (!canRun);
    }

    @Override
    public void init() {
        try {
            autoData = new File(SmartDashboard.getString("autofile", "auto-1.json"));
            is = new FileInputStream(autoData);
            json = new JsonParser().parse(new InputStreamReader(is));

            Drive.INSTANCE.setUseGyro(false);
            Drive.INSTANCE.setUseVelocityPid(true);
        } catch(Exception e) {
            System.err.println("Error loading auto file: " + e.getMessage());
            System.err.println("Disabling auto");
            canRun = false;
        }
    }

    @Override
    public void update() {
        try {
            JsonObject currentVelocities = json.getAsJsonObject().getAsJsonObject("" + getDeltaTime());
            double leftVel = currentVelocities.get("left_rpm").getAsDouble();
            double rightVel = currentVelocities.get("right_rpm").getAsDouble();

            Drive.INSTANCE.setLeftSetpoint(leftVel);
            Drive.INSTANCE.setRightSetpoint(rightVel);
        } catch (Exception e) {
            System.out.println("Error while playing: " + e.getMessage());
            canRun = false;
        }
    }
}
