package frc.robot.shooting;

public class ShootingState {
    public final double distanceFromTarget;
    public final double flyWheelRPM;
    public final double hoodAngleDegrees;

    public ShootingState(double distanceFromTarget, double flyWheelRPM, double hoodAngleDegrees){
        this.distanceFromTarget = distanceFromTarget;
        this.flyWheelRPM = flyWheelRPM;
        this.hoodAngleDegrees = hoodAngleDegrees;
    }
    
}
