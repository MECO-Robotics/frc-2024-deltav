package frc.robot.subsystems;
/* 
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
    private PhotonCamera m_camera; 
    public PhotonPipelineResult result;
    public PhotonPoseEstimator m_photonEstimator;

    public Limelight(String cameraName, Transform3d RobotToCam) {
        m_camera = new PhotonCamera(cameraName);
        m_photonEstimator = new PhotonPoseEstimator(
            Constants.Vision.kTagLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            m_camera,
            RobotToCam);

        m_photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        SmartDashboard.putNumber("LL Pipeline Index", m_camera.getPipelineIndex());
        //m_camera.setLED(VisionLEDMode.kBlink);
    }

    @Override
    public void periodic() {
        result = m_camera.getLatestResult();
    }

    public boolean hasTarget() {
        return result.hasTargets();
    }

    public Command setLED(VisionLEDMode ledMode) {
        return runOnce(() -> m_camera.setLED(ledMode));
    }

    public Optional<EstimatedRobotPose> getEstimatedPose() {
        Optional<EstimatedRobotPose> estPose = m_photonEstimator.update();
        return estPose;
    }

    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
       // var estStdDevs = Constants.Vision.kSingleTagStdDevs;
        var targets = result.getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = m_photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                tagPose
                    .get()
                    .toPose2d()
                    .getTranslation()
                    .getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        if (numTags > 1) estStdDevs = Constants.Vision.kMultiTagStdDevs;
        if (numTags == 1 && avgDist > 4) estStdDevs =
            VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE); else estStdDevs =
            estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }
}
*/ 