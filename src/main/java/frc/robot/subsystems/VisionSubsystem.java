package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    // Step 1: Uncomment this class variable delcaration:
    // PhotonCamera camera = new PhotonCamera("photonvision");

    // Step 2: write a function that gets the rotation to the aim at the speaker.
    // Should provide both pitch and yaw.
    Rotation3d getRotationToSpeaker() {
        Rotation3d rotation = new Rotation3d();
        return rotation;
    }

    // Step
    float getRangeToSpeaker() {
        return 0.0f;
    }
}
