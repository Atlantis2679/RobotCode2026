package frc.robot;

public final class RobotMap {
  public static final class ModuleFL {
    public final static int DRIVE_MOTOR_ID = 20;
    public final static int TURN_MOTOR_ID = 21;
    public final static int CAN_CODER_ID = 50;
  }

  public static final class ModuleFR {
    public final static int DRIVE_MOTOR_ID = 22;
    public final static int TURN_MOTOR_ID = 23;
    public final static int CAN_CODER_ID = 51;
  }

  public static final class ModuleBL {
    public final static int DRIVE_MOTOR_ID = 26;
    public final static int TURN_MOTOR_ID = 27;
    public final static int CAN_CODER_ID = 53;
  }

  public static final class ModuleBR {
    public final static int DRIVE_MOTOR_ID = 24;
    public final static int TURN_MOTOR_ID = 25;
    public final static int CAN_CODER_ID = 52;
  }

  public static final class Controllers {
    public static final int DRIVER_PORT = 0;
  }

  public static final class CANBUS {
    public static final int SPINDEX_ID = 0;
    public static final int INDEXER_ID = 0;

    public static final int ELEVATOR_ID = 0;
    public static final int PIVOT_ID = 0;

    public static final int SLAPDOWN_ID = 0;
    public static final int FLYWHEEL_MOTOR1_ID = 0;
    public static final int FLYWHEEL_MOTOR2_ID = 0;
    public static final int ROLLER_ID = 0;

    public final static int HOOD_MOTOR_ID = 0;
  }

  public static final class DIO {
    public static final int SLAPDOWN_ENCODER_ID = 0;
    public final static int HOOD_ENCODER_ID = 0;
    public static final int ELEVATOR_ENCODER_ID = 0;
    public static final int PIVOT_ENCODER_ID = 0;
  }
}
