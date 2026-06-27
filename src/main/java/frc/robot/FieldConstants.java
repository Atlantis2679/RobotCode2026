package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;

public class FieldConstants {
    public static final double HEIGHT_ABOVE_FIELD_THREASHOLD_METERS = 0.4;
    public static final double PITCH_ROLL_THREASHOLD_DEGRESS = 10;

    public static final double HUB_OPENING_HEIGHT_METERS = 1.8288;

    public static boolean isOnField(Pose3d pose) {
        if (Math.abs(pose.getZ()) > HEIGHT_ABOVE_FIELD_THREASHOLD_METERS) return false;
        if (Math.toDegrees(Math.abs(pose.getRotation().getX())) > PITCH_ROLL_THREASHOLD_DEGRESS) return false;
        if (Math.toDegrees(Math.abs(pose.getRotation().getY())) > PITCH_ROLL_THREASHOLD_DEGRESS) return false;        
        return true;
    }
}

