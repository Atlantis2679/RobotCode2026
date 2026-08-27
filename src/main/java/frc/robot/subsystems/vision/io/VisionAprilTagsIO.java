package frc.robot.subsystems.vision.io;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class VisionAprilTagsIO extends IOBase {
    public record VisionData(
        double timestamp,
        Pose3d robotPose,
        Pose3d[] tagsPoses,
        double ambiguity,
        double[] tagsDistancesToCam
    ) implements StructSerializable {
        public static final Struct<VisionData> struct = StructGenerator.genRecord(VisionData.class);
    }

    public final BooleanSupplier isConnected = fields.addBoolean("isConnected", this::getIsConnected);
    public final Supplier<VisionData[]> visionData = fields.addObjectArray("visionData", this::visionData, new VisionData[0]); 

    protected VisionAprilTagsIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    public abstract CameraConfig getCameraConfig();

    protected abstract boolean getIsConnected();

    protected abstract VisionData[] visionData();
}
