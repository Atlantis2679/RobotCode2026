package frc.robot.subsystems.vision.io;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class VisionAprilTagsIO extends IOBase {
    public final Supplier<Pose3d[]> posesEstimates = fields.addObjectArray("poseEstimates", this::getRobotPoses, new Pose3d[0]);
    public final Supplier<double[]> cameraTimestampsSeconds = fields.addDoubleArray("cameraTimestampsSeconds",
            this::getCameraTimestampsSeconds);
    public final Supplier<Pose3d[][]> tagsPoses = fields.addObjectMatrix("tagsPoses", this::getTagsPoses, new Pose3d[0][0]);
    public final Supplier<double[][]> tagsDistanceToCam = fields.addDoubleMatrix("tagsDistanceToCam", this::getTagsDistanceToCam);
    public final Supplier<double[]> tagsAmbiguities = fields.addDoubleArray("tagsAmbiguities", this::getTagsAmbiguities);
    public final BooleanSupplier isConnected = fields.addBoolean("isConnected", this::getIsConnected);

    protected VisionAprilTagsIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    public abstract CameraConfig getCameraConfig();

    protected abstract double[] getCameraTimestampsSeconds();

    protected abstract Pose3d[] getRobotPoses();

    protected abstract Pose3d[][] getTagsPoses();

    protected abstract double[] getTagsAmbiguities();

    protected abstract double[][] getTagsDistanceToCam();

    protected abstract boolean getIsConnected();
}
