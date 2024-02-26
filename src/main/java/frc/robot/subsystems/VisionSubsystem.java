package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {

    // Step 1: Uncomment this class variable delcaration:
    // PhotonCamera camera = new PhotonCamera("photonvision");
    

    // Step 2: write a function that gets the rotation to the aim at the speaker.
    // Should provide both pitch and yaw.
    double getRotationToSpeaker() {
        //return LimelightHelpers.getTX("limelight");
        return 10.0f;
    }

    // Step
    float getRangeToSpeaker() {
        return .5f;
    }





    private void sampleCodeFromLimelightDocumentation() {

        //
        // The following code was pulled from https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance
        //

        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ty = table.getEntry("ty");
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);
    
        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 25.0; 
    
        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 20.0; 
    
        // distance from the target to the floor
        double goalHeightInches = 60.0; 
    
        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
    
        //calculate distance
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);


    }
      

}
