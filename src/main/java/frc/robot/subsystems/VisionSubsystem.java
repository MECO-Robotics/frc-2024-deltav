package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {

    // Step 1: Uncomment this class variable delcaration:
    // PhotonCamera camera = new PhotonCamera("photonvision");

    /**
     * Get the number of horizontal degrees difference between the direction the
     * camera is pointing and where the speaker april tag is.
     * 
     * @return The angle, in degrees between -29.8 and 29.9
     */
    public double getHorizontalRotationToSpeaker() {

        // tx = Horizontal Offset From Crosshair To Target (LL2: -29.8 to 29.8 degrees)
        // return LimelightHelpers.getTX("limelight");

        // PDL:
        // IF Tag is found THEN
        // Set the isValid flag to true
        // RETURN the tx value
        return 10.0f;
    }

    /**
     * Get the number of vertical degrees difference between the direction the
     * camera is pointing and where the speaker april tag is.
     * 
     * @return The angle, in degrees between -24.85 and 24.85
     */
    public double getVerticalRotationToSpeaker() {

        // TODO Call the function to get the current tag id

        // TODO get ty - Vertical Offset From Crosshair To Target (LL2: -24.85 to 24.85
        // degrees)
        // Get it for the current tag id

        // PDL:
        // IF Tag is found THEN
        // Set the isValid flag to true
        // RETURN the ty value
        // ELSE
        // Set the isValid flag to false
        // RETURN 0.0;
        return .5f;
    }

    // TODO Write a function that gets the current alliance and returns the proper
    // tag id based on the alliance.

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        // The network tables are already available for the limelight, but these values are only non-zero when the april tag is in view.

        builder.addDoubleProperty("forward-camera-tx", this::getHorizontalRotationToSpeaker, null);
        builder.addDoubleProperty("forward-camera-ty", this::getVerticalRotationToSpeaker, null);
    }
}
