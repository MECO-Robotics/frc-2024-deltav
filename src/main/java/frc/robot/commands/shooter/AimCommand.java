package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * Turn the robot to point to the Speaker, and elevate the arm based on the
 * distance to the speaker.
 */
public class AimCommand extends Command {

    private final VisionSubsystem vision;
    private final SwerveSubsystem swerve;
    private final ArmSubsystem arm;

    public AimCommand(VisionSubsystem visionPassedIn, SwerveSubsystem swerveDrive, ArmSubsystem armPassedIn) {
        vision = visionPassedIn;
        swerve = swerveDrive;
        arm = armPassedIn;
    }

    // TODO execute():
    // 1. Get the RotationToSpeaker from the Vision Subsystem
    // 2. Convert the angle to a angular rotation velocity in radians/sec using a
    // simple proportional constant or full PIDF control
    // 3. Call the drive method on the SwerveSubsystem with 0 translation and the
    // rotation just calculated
    // 4. Get the RangeToSpeaker from the Vision Subsystem
    // 5. Convert the range to speaker from a distance to a arm position using a
    // look up table or proportional constant
    // 6. Set the arm position using the appropriate method on the Arm Subsystem

    // TODO isFinished():
    // 1. Get the Rotation and range from the Vision Subsystem
    // 2. Do the same conversions as done in execute()
    // 3. return true if both the error in angular velocity and desired arm position
    // is close to zero.
    // return false otherwise.

    /*
     * public void calculate() {
     * 
     * }
     */

    public void execute() {
        // Drive
        double horizontalAngle = vision.getHorizontalRotationToSpeaker();
        Translation2d noTranslation = new Translation2d(0, 0);
        swerve.drive(noTranslation, horizontalAngle * 0.1, true);
        // write angular position command for swerve instead of angular velocity?

        // Arm
        double verticalAngle = vision.getVerticalRotationToSpeaker();
        // TODO Use a polynomial regression to convert the angle to target to a armposition (in encoder ticks) 
        double aimPosition = verticalAngle * 0.1;
        arm.aimSpeaker(aimPosition);
        // aim speaker

    }

    public void end(boolean interrupted) {

    }

    public boolean isFinished() {
        // placeholder
        return false;
    }

}
