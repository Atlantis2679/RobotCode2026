package frc.robot.subsystems.vision;

import team2679.atlantiskit.tunables.Tunable;
import team2679.atlantiskit.tunables.TunableBuilder;

public class TunableTrustLevel implements Tunable {
  private TrustLevel trustLevel;

  public TunableTrustLevel(TrustLevel trustLevel) {
    this.trustLevel = trustLevel;
  }

  public TrustLevel get() {
    return trustLevel;
  }

  public void set(TrustLevel trustLevel) {
    this.trustLevel = trustLevel;
  }

  @Override
  public void initTunable(TunableBuilder builder) {
    builder.addDoubleProperty("xyStdDev", () -> trustLevel.xyStdDev(), (xyStdDev) -> set(new TrustLevel(xyStdDev, trustLevel.rotationStdDev())));
    builder.addDoubleProperty("rotationStdDev", () -> trustLevel.rotationStdDev(), (rotationStdDev) -> set(new TrustLevel(trustLevel.xyStdDev(), rotationStdDev)));
  }
}
