package frc.robot;

public final class RobotMap {
  public static final class ModuleFL {
    public final static int DRIVE_MOTOR_ID = 0;
    public final static int TURN_MOTOR_ID = 1;
    public final static int CAN_CODER_ID = 10;
  }

  public static final class ModuleFR {
    public final static int DRIVE_MOTOR_ID = 2;
    public final static int TURN_MOTOR_ID = 3;
    public final static int CAN_CODER_ID = 11;
  }

  public static final class ModuleBL {
    public final static int DRIVE_MOTOR_ID = 4;
    public final static int TURN_MOTOR_ID = 5;
    public final static int CAN_CODER_ID = 12;
  }

  public static final class ModuleBR {
    public final static int DRIVE_MOTOR_ID = 6;
    public final static int TURN_MOTOR_ID = 7;
    public final static int CAN_CODER_ID = 13;
  }

  public static final class Controllers {
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
  }

  public static final class CANBUS {
    public static final int SPINDEX_LEADER_ID = 40;
    public static final int INDEXER_ID = 41;
    public static final int SPINDEX_FOLLOWER_ID = 42;

    public static final int ELEVATOR_ID = 0;

    public static final int FOURBAR_ID = 30;
    public static final int FLYWHEEL_MOTOR1_ID = 20;
    public static final int FLYWHEEL_MOTOR2_ID = 21;
    public static final int ROLLER_ID = 31;

    public final static int HOOD_MOTOR_ID = 22;
  }

  public static final class DIO {
    public static final int FOURBAR_ENCODER_ID = 0;
    public static final int ELEVATOR_ENCODER_ID = 0;
  }
}
