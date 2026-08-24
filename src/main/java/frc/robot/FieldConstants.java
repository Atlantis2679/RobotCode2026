package frc.robot;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose3d;

public class FieldConstants {
    public static final double HEIGHT_ABOVE_FIELD_THRESHOLD_METERS = 0.4;
    public static final double PITCH_ROLL_THRESHOLD_DEGRESS = 10;

    public static boolean isOnField(Pose3d pose) {
        if (Math.abs(pose.getZ()) > HEIGHT_ABOVE_FIELD_THRESHOLD_METERS) return false;
        if (Math.toDegrees(Math.abs(pose.getRotation().getX())) > PITCH_ROLL_THRESHOLD_DEGRESS) return false;
        if (Math.toDegrees(Math.abs(pose.getRotation().getY())) > PITCH_ROLL_THRESHOLD_DEGRESS) return false;  
        if (pose.getX() < 0 || pose.getY() < 0) return false;
        if (pose.getX() > FlippingUtil.fieldSizeX || pose.getY() > FlippingUtil.fieldSizeY) return false;
        return true;
    }
}

