package frc.robot.commands.swervedrive;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class LimelightDefaultCommand extends Command {
    private Pose2d botPose;
    private double[] botPoseArray;
    SwerveDrivePoseEstimator poseEstimator;
    LimelightSubsystem limelightSubsystem;
    SwerveSubsystem swerveSubsystem;

    public LimelightDefaultCommand(LimelightSubsystem limelightSubsystem, SwerveSubsystem swerveSubsystem) {
        this.limelightSubsystem = limelightSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(limelightSubsystem);
    }

    @Override
    public void initialize() {
        limelightSubsystem.setPipeline(0);
        limelightSubsystem.setPriorityID(-1);
    }

    @Override
    public void execute() {
        if (limelightSubsystem.getTv()) {
            botPoseArray = limelightSubsystem.getBotPose();
            if (swerveSubsystem.isRedAlliance()) {
                botPose = new Pose2d(botPoseArray[0], botPoseArray[1],
                        swerveSubsystem.getPose().getRotation().plus(Rotation2d.fromDegrees(180)));
            } else {
                botPose = new Pose2d(botPoseArray[0], botPoseArray[1],
                        swerveSubsystem.getPose().getRotation());
            }
            //swerveSubsystem.addVisionMeasurement(botPose, Timer.getFPGATimestamp(), );

            limelightSubsystem.updatePoseEstimatorWithVisionBotPose(
                    swerveSubsystem.getEstimator(), botPose);

        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}