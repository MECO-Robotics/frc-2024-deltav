package frc.robot.commands.vision;

import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

public class DriveWithVisionRotation extends Command {

    private final VisionSubsystem vision;
    private final SwerveSubsystem swerve;
    protected DoubleSupplier vX, vY;
    protected DoubleSupplier headingHorizontal, headingVertical;

    public DriveWithVisionRotation(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY,
            VisionSubsystem vision) {

        super();

        this.vision = vision;
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;

        addRequirements(swerve, vision);
    }

    public void execute() {

        double desiredAngle = swerve.getHeading().getDegrees() - vision.getRotationErrorAngle();
        double desiredAngleRads = desiredAngle * 2.0 * Math.PI / 360.0;

        // Get the desired chassis speeds based on a 2 joystick module.
        var desiredSpeeds = swerve.getSwerveController().getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(), desiredAngleRads,
                swerve.getHeading().getRadians(), 10.0);

        // Limit velocity to prevent tippy
        Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
        translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
                Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS),
                swerve.getSwerveDriveConfiguration());

        // Make the robot move
        swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);
    }
}
