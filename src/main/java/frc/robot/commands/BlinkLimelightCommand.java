package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class BlinkLimelightCommand {
    private double startTime;
    private boolean done = false;

    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2);
    }

    public void execute() {
        if (Timer.getFPGATimestamp() > (startTime + 5)) {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
            done = true;
        }
    }

    public boolean isFinished() {
        return done;
    }

}