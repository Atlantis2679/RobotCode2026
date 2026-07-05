package frc.robot;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldConstants {
    public static final double HEIGHT_ABOVE_FIELD_THREASHOLD_METERS = 0.4;
    public static final double PITCH_ROLL_THREASHOLD_DEGRESS = 10;

    public static final double HUB_OPENING_HEIGHT_METERS = 1.8288;

    public static final Pose2d STARTING_POSE_BLUE = new Pose2d(new Translation2d(3.5, 1.0), new Rotation2d(Math.toRadians(85)));
    public static final Pose2d STARTING_POSE_RED = FlippingUtil.flipFieldPose(STARTING_POSE_BLUE);

    public static boolean isOnField(Pose3d pose) {
        if (Math.abs(pose.getZ()) > HEIGHT_ABOVE_FIELD_THREASHOLD_METERS) return false;
        if (Math.toDegrees(Math.abs(pose.getRotation().getX())) > PITCH_ROLL_THREASHOLD_DEGRESS) return false;
        if (Math.toDegrees(Math.abs(pose.getRotation().getY())) > PITCH_ROLL_THREASHOLD_DEGRESS) return false;        
        return true;
    }
}

