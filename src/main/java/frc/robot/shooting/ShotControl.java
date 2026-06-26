package frc.robot.shooting;

import static frc.robot.shooting.ShotConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.shooting.ShotCalculator.LaunchParameters;
import frc.robot.shooting.ShotCalculator.ShotInputs;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.tunables.Tunable;
import team2679.atlantiskit.tunables.TunableBuilder;
import team2679.atlantiskit.valueholders.DoubleHolder;

public class ShotControl implements Tunable {
    private final ProjectileSimulator sim;
    
    private final ShotLUT lut;

    private final ShotCalculator shotCalculator;

    private DoubleHolder shotConfidenceFilter = new DoubleHolder(SHOT_CONFIDENCE_FILTER_THRESHOLD);

    private LaunchParameters shotParams = LaunchParameters.INVALID;

    private final LogFieldsTable fieldsTable = new LogFieldsTable("Shot Control");

    private boolean shoot = false;

    private double rpmAdjusment = 0;
    private double hoodAdjusment = 0;

    public ShotControl() {
        sim = new ProjectileSimulator(PARAMETERS);
        lut = sim.generateVariableAngleShotLUT(MIN_ANGLE_DEG, MAX_ANGLE_DEG, ANGLE_STEP);
        shotCalculator = new ShotCalculator(CONFIG);
    }

    public void update(Pose2d robotPose, boolean isRedAlliance, ChassisSpeeds fieldSpeeds, ChassisSpeeds robotSpeeds, double robotPitchDeg, double robotRollDeg) {
        shotCalculator.loadShotLUT(lut);
        Translation2d target = isRedAlliance ? RED_HUB : BLUE_HUB;
        Translation2d targetHeading = isRedAlliance ? RED_HUB_HEADING : BLUE_HUB_HEADING;
        ShotInputs inputs = new ShotInputs(robotPose, fieldSpeeds, robotSpeeds, target,
            targetHeading, POSE_CONFIDENCE);
        fieldsTable.recordOutput("Inputs", inputs);
        LaunchParameters nextShot = shotCalculator.calculate(inputs);
        if (nextShot.isValid()) {
            this.shotParams = nextShot;
            shoot = nextShot.confidence() > shotConfidenceFilter.get();
        }
        fieldsTable.recordOutput("Shot Params", shotParams);
        fieldsTable.recordOutput("Shoot", shoot);
        SmartDashboard.putNumber("Shot RPM adjustment", rpmAdjusment);
        SmartDashboard.putNumber("Shot Hood angle adjustment", hoodAdjusment);
    }

    public void adjustRpmOffset(double rpm) {
        rpmAdjusment += rpm;
        shotCalculator.adjustOffset(rpm);
    }

    public void adjustHoodOffset(double angle) {
        hoodAdjusment += angle;
    }

    public double getRpm() {
        return shotParams.rpm();
    }

    public double getAngle() {
        return shotCalculator.getHoodAngle(shotParams.solvedDistanceM()) + hoodAdjusment;
    }

    public double getDriveRotationRPS() {
        return shotParams.driveAngularVelocityRadPerSec() / (2 * Math.PI);
    }

    public double getDriveAngleDegrees() {
        return shotParams.driveAngle().getDegrees();
    }

    public boolean shoot() {
        return shoot;
    }

    @Override
    public void initTunable(TunableBuilder builder) {
        builder.addDoubleProperty("Shot Confidence Filter", shotConfidenceFilter::get, shotConfidenceFilter::set);
    }
}
