package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/** 
 * Turn the robot to point to the Speaker, and elevate the arm based on the distance to the speaker.
 */
public class exampleCommand extends ParallelCommandGroup {

    VisionSubsystem vision;

    public exampleCommand (VisionSubsystem vision, SwerveSubsystem swerve, ArmSubsystem arm, DoubleSupplier x, DoubleSupplier y){
        this.vision = vision;
        this.addCommands(
            //new ArmCommand(0/*arm pos*/, 3000 /*flywhee*/ ),
            new TeleopDrive(swerve, x, y, this::getDriveAngle, () -> true));
    }

    private double getDriveAngle(){
        // Create calculations to get angle of drive
        return vision.getRotationToSpeaker() /*drivetrain something*/;
    }

    private double getArmAngle() {

        return vision.getRangeToSpeaker();
    }


}