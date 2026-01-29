package frc.robot.shooting;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

import frc.robot.utils.LinearInterpolation;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class ShootingCalculator {
    private static final double G = 9.8;

    private final LogFieldsTable fieldsTable = new LogFieldsTable("ShootingCalculations");

    private final LinearInterpolation hoodAngleDegreesLinearInterpolation;
    private final LinearInterpolation flyWheelRPMLinearInterpolation;

    private double robotYawDegreesCCW;

    private double hoodAngleDegrees;
    private double flyWheelRPM;

    private double flightTimeEstimateSeconds = 0;

    private Pose3d targetPose;

    public ShootingCalculator(Pose3d targetPose, ShootingState[] shootingStates) {
        List<LinearInterpolation.Point> hoodAngleDegreesPoints = new ArrayList<>();
        List<LinearInterpolation.Point> flyWheelRPMPoints = new ArrayList<>();

        for (ShootingState shootingState : shootingStates) {
            hoodAngleDegreesPoints.add(
                    new LinearInterpolation.Point(shootingState.distanceFromTarget(),
                            shootingState.hoodAngleDegrees()));
            flyWheelRPMPoints.add(
                    new LinearInterpolation.Point(shootingState.distanceFromTarget(), shootingState.flyWheelRPM()));
        }

        hoodAngleDegreesLinearInterpolation = new LinearInterpolation(hoodAngleDegreesPoints);
        flyWheelRPMLinearInterpolation = new LinearInterpolation(flyWheelRPMPoints);

        this.targetPose = targetPose;
    }

    public void update(Pose2d robotPose, boolean isRedAlliance) {
        double distanceFromTarget = new Pose3d(robotPose).getTranslation().getDistance(targetPose.getTranslation());
        robotYawDegreesCCW = Math.toDegrees(Math
                .atan((targetPose.getY() - robotPose.getY()) / (targetPose.getX() - robotPose.getX())));
        if (isRedAlliance) {
            robotYawDegreesCCW += 180;
        }

        hoodAngleDegrees = hoodAngleDegreesLinearInterpolation.calculate(distanceFromTarget);
        flyWheelRPM = flyWheelRPMLinearInterpolation.calculate(distanceFromTarget);

        flightTimeEstimateSeconds = solveKinematicsTime(targetPose.getX(), flyWheelRPM, hoodAngleDegrees);

        fieldsTable.recordOutput("distanceFromTarget", distanceFromTarget);
        fieldsTable.recordOutput("robotYawDegreesCCW", robotYawDegreesCCW);
        fieldsTable.recordOutput("hoodAngleDegrees", hoodAngleDegrees);
        fieldsTable.recordOutput("flyWheelRPM", flyWheelRPM);
        fieldsTable.recordOutput("flightTimeEstimateSeconds", flightTimeEstimateSeconds);
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

    public double getFlightTimeEstimateSeconds() {
        return flightTimeEstimateSeconds;
    }

    private static double solveKinematicsTime(double deltaHeight, double startRPM, double angleDegrees) {
        return solveRealQuadratic(0.5 * G, Math.sin(angleDegrees) * startRPM, -deltaHeight);
    }

    private static double solveRealQuadratic(double a, double b, double c) {
        double discriminant = b * b - 4 * a * c;
        double root1 = -b + Math.sqrt(discriminant) / (2 * a);
        double root2 = -b + Math.sqrt(discriminant) / (2 * a);
        if (root1 > 0) return root1;
        return root2;
    }
}
