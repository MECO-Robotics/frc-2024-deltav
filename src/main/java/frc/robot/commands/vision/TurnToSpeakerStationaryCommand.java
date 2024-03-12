package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class TurnToSpeakerStationaryCommand extends Command {

    private final SwerveSubsystem swerve;
    private final VisionSubsystem vision;

    public TurnToSpeakerStationaryCommand(SwerveSubsystem swerveSub, VisionSubsystem visionSub) {
        swerve = swerveSub;
        vision = visionSub;

        System.out.println("RUNNING TurnToSpeakerCommand");
        addRequirements(swerve, vision);
    }

    public void initialize() {
        
    }

    public void execute() {
        // Vision angles are positve CW in degrees, the rotation speed is rad/s CCW
        // positive
        double desiredAngle = swerve.getHeading().getDegrees() - vision.getRotationErrorAngle();
        double desiredAngleRads = desiredAngle * 2.0 * Math.PI / 360.0;
        // System.out.println("AIMING - HEADING: " + swerve.getHeading().getDegrees());
        // System.out.println("AIMING - ERROR: " + vision.getRotationErrorAngle());
        // System.out.println("AIMING - DESIRED: " + desiredAngle);
        
        var chassisSpeeds = swerve.getSwerveController().getTargetSpeeds(0, 0, desiredAngleRads, swerve.getHeading().getRadians(), 10.0);

        swerve.drive(chassisSpeeds);
    }

    public void end(boolean interrupted) {
        System.out.println("YAW - :" + swerve.getSwerveController().lastAngleScalar);
        swerve.getSwerveController().lastAngleScalar = ((swerve.getHeading().getDegrees()
                + vision.getRotationErrorAngle()) % 360) * 2.0 * Math.PI / 360.0;
        System.out.println("YAW AFTER - :" + swerve.getSwerveController().lastAngleScalar);

    }

    public boolean isFinished() {
        // return Math.abs(vision.getRotationErrorAngle()) < 3;
        return false;
    }
}