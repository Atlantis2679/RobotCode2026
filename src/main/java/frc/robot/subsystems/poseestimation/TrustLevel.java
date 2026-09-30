package frc.robot.subsystems.poseestimation;

public record TrustLevel(double xyStdDev, double rotationStdDev) {
  public TrustLevel multiply(TrustLevel other)  {
    return new TrustLevel(this.xyStdDev * other.xyStdDev, this.rotationStdDev * other.rotationStdDev);
  }
}
