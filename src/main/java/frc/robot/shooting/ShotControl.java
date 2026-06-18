package frc.robot.shooting;

import static frc.robot.shooting.ShotConstants.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.shooting.ShotCalculator.LaunchParameters;
import frc.robot.shooting.ShotCalculator.ShotInputs;
import frc.robot.subsystems.poseestimation.PoseEstimator;

public class ShotControl {
    private static final ShotControl instance = new ShotControl();

    private final ProjectileSimulator sim = new ProjectileSimulator(PARAMETERS);

    private final ShotCalculator shotCalculator = new ShotCalculator(CONFIG);

    private final ShotLUT lut = sim.generateVariableAngleShotLUT(MIN_ANGLE_DEG, MAX_ANGLE_DEG, ANGLE_STEP);

    private ShotControl() {
        shotCalculator.loadShotLUT(lut);
    }

    public static ShotControl getInstance() {
        return instance;
    }

    public void update(boolean isRedAlliance, ChassisSpeeds fieldSpeeds, ChassisSpeeds robotSpeeds, double robotPitchDeg, double robotRollDeg) {
        Translation2d target = isRedAlliance ? RED_HUB : BLUE_HUB;
        Translation2d targetHeading = isRedAlliance ? RED_HUB_HEADING : BLUE_HUB_HEADING;
        ShotInputs inputs = new ShotInputs(
            PoseEstimator.getInstance().getEstimatedPose(),
            fieldSpeeds, robotSpeeds, target, targetHeading, POSE_CONFIDENCE, robotPitchDeg, robotRollDeg);
        LaunchParameters shot = shotCalculator.calculate(inputs);
        if (shot.isValid() && shot.confidence() > SHOT_CONFIDENCE_FILTER_THRESHOLD) {
            shot.driveAngularVelocityRadPerSec()
        }
    }
}
