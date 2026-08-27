package frc.robot.subsystems.poseestimation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import frc.robot.subsystems.vision.VisionConstants;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.tunables.Tunable;
import team2679.atlantiskit.tunables.TunableBuilder;

public class OdometryDriftSim implements Tunable {
    private LogFieldsTable fieldsTable;
    private Pose2d realPose = new Pose2d();
    private Pose2d driftSimPose = new Pose2d();
    private double translationDriftFactor;
    private double rotationDriftFactor;

    public OdometryDriftSim(double translationDriftFactor, double rotationDriftFactor, Pose2d startingPose, LogFieldsTable parentTable) {
        this.realPose = startingPose;
        this.fieldsTable = parentTable.getSubTable("Sim odometry drift");
        this.translationDriftFactor = translationDriftFactor;
        this.rotationDriftFactor = rotationDriftFactor;
    }

    public Twist2d process(Twist2d trueTwist) {
        realPose = realPose.exp(trueTwist);
        fieldsTable.recordOutput("Real Pose", realPose);
        VisionConstants.Sim.VISION_SIM.update(realPose);
        fieldsTable.recordOutput("Drifted Pose", driftSimPose);
        Twist2d driftedTwist = new Twist2d(
                trueTwist.dx * translationDriftFactor,
                trueTwist.dy * translationDriftFactor,
                trueTwist.dtheta * rotationDriftFactor);
        driftSimPose = driftSimPose.exp(driftedTwist);
        return driftedTwist;
    }

    public void reset(Pose2d pose) { realPose = pose; }

    public void recordError(Pose2d estimatedPose) {
        fieldsTable.recordOutput("Translation Error",
                realPose.getTranslation().getDistance(estimatedPose.getTranslation()));
        fieldsTable.recordOutput("Rotation Error Deg",
                realPose.getRotation().minus(estimatedPose.getRotation()).getDegrees());
    }

    public double getTranslationDriftFactor() {
        return this.translationDriftFactor;
    }

    public double getRotationDriftFactor() {
        return rotationDriftFactor;
    }

    public void setTranslationDriftFactor(double translationDriftFactor) {
        this.translationDriftFactor = translationDriftFactor;
    }

    public void setRotationDriftFactor(double rotationDriftFactor) {
        this.rotationDriftFactor = rotationDriftFactor;
    }

    @Override
    public void initTunable(TunableBuilder builder) {
        builder.addDoubleProperty("transition drift factor", this::getTranslationDriftFactor, this::setTranslationDriftFactor);
        builder.addDoubleProperty("rotation drift factor", this::getRotationDriftFactor, this::setRotationDriftFactor);
    }
}