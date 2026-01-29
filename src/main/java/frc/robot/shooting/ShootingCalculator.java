package frc.robot.shooting;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;

import static frc.robot.shooting.ShootingMeasurments.*;
import frc.robot.utils.LinearInterpolation;
import team2679.atlantiskit.logfields.LogFieldsTable;
import static frc.robot.Constants.*;

public class ShootingCalculator {
    private final LogFieldsTable fieldsTable = new LogFieldsTable("ShootingCalculations");

    private final LinearInterpolation hoodAngleDegreesLinearInterpolation;
    private final LinearInterpolation flyWheelRPMLinearInterpolation;

    private double robotYawDegreesCCW;

    private double hoodAngleDegrees;
    private double flyWheelRPM;

    private boolean isShootingHub;

    public ShootingCalculator(boolean isShootingHub) {
        List<LinearInterpolation.Point> hoodAngleDegreesPoints = new ArrayList<>();
        List<LinearInterpolation.Point> flyWheelRPMPoints = new ArrayList<>();

        for (ShootingState shootingState : isShootingHub ? ALL_MEASURMENTS_HUB : ALL_MEASURMENTS_DELIVRY) {
            hoodAngleDegreesPoints.add(
                    new LinearInterpolation.Point(shootingState.distanceFromTarget(),
                            shootingState.hoodAngleDegrees()));
            flyWheelRPMPoints.add(
                    new LinearInterpolation.Point(shootingState.distanceFromTarget(), shootingState.flyWheelRPM()));
        }

        hoodAngleDegreesLinearInterpolation = new LinearInterpolation(hoodAngleDegreesPoints);
        flyWheelRPMLinearInterpolation = new LinearInterpolation(flyWheelRPMPoints);

        this.isShootingHub = isShootingHub;
    }

    public void update(Pose2d robotPose, boolean isRedAlliance) {
        Pose2d targetPose;
        if (isShootingHub) {
            targetPose = isRedAlliance ? FlippingUtil.flipFieldPose(BLUE_HUB_POSE) : BLUE_HUB_POSE;
        } else {
            targetPose = isRedAlliance ? FlippingUtil.flipFieldPose(BLUE_DELIVERY_POSE) : BLUE_DELIVERY_POSE;
        }
        double distanceFromTarget = 
        robotYawDegreesCCW = Math.toDegrees(Math
                .atan((targetPose.getY() - robotPose.getY()) / (targetPose.getX() - robotPose.getX())));
        if (isRedAlliance) {
            robotYawDegreesCCW += 180;
        }

        hoodAngleDegrees = hoodAngleDegreesLinearInterpolation.calculate(distanceFromTarget);
        flyWheelRPM = flyWheelRPMLinearInterpolation.calculate(distanceFromTarget);

        fieldsTable.recordOutput("distanceFromTarget", distanceFromTarget);
        fieldsTable.recordOutput("robotYawDegreesCCW", robotYawDegreesCCW);
        fieldsTable.recordOutput("hoodAngleDegrees", hoodAngleDegrees);
        fieldsTable.recordOutput("flyWheelRPM", flyWheelRPM);
    }

    public double getHoodAngleDegrees() {
        return hoodAngleDegrees;
    }

    public double getFlyWheelRPM() {
        return flyWheelRPM;
    }

    public double getRobotYawDegreesCCW() {
        return robotYawDegreesCCW;
    }
}
