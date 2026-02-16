package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;

public class FieldConstants {
    public static final double HEIGHT_ABOVE_FIELD_THREASHOLD_METERS = 0.1;

    public static boolean isOnField(Pose3d pose) {
        // Check height:
        if (Math.abs(pose.getZ()) > HEIGHT_ABOVE_FIELD_THREASHOLD_METERS) return false;

        // Check boundings:
        
        return true;
    }
}

