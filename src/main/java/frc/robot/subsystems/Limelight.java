package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        System.out.println("Getting Latest Result");
    }

    public boolean hasTarget() {
        return result.hasTargets();
    }

    public Optional<EstimatedRobotPose> getEstimatedPose() {
        Optional<EstimatedRobotPose> estPose = m_photonEstimator.update();
        System.out.println(estPose.toString());
        return estPose;
    }
}
