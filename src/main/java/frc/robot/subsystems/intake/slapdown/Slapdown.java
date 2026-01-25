package frc.robot.subsystems.intake.slapdown;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.intake.slapdown.io.SlapdownIO;
import frc.robot.subsystems.intake.slapdown.io.SlapdownIOSim;
import frc.robot.subsystems.intake.slapdown.io.SlapdownIOSparkMax;
import team2679.atlantiskit.helpers.RotationalSensorHelper;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.tunables.extensions.TunableArmFeedforward;
import team2679.atlantiskit.tunables.extensions.TunableTrapezoidProfile;

public class Slapdown extends SubsystemBase {
    private Debouncer encoderConnectedDebouncer = new Debouncer(SlapdownConstants.DEBOUNCER_DELAY);
    private TunableArmFeedforward feedforward = new TunableArmFeedforward(SlapdownConstants.ks, SlapdownConstants.kg, SlapdownConstants.kv);
    private TunableTrapezoidProfile trapezoidProfile = new TunableTrapezoidProfile(
        new Constraints(SlapdownConstants.MAX_VELOCITY, SlapdownConstants.MAX_ACCELERATION));
    private PIDController pid = new PIDController(SlapdownConstants.kp, SlapdownConstants.ki, SlapdownConstants.kd);
    private LogFieldsTable fieldsTable = new LogFieldsTable(getName());
    private SlapdownIO io = Robot.isReal() ? new SlapdownIOSparkMax(fieldsTable) : new SlapdownIOSim(fieldsTable);
    private RotationalSensorHelper sensorHelper;

    public Slapdown(){
        sensorHelper = new RotationalSensorHelper(getAngleDegrees(), SlapdownConstants.ANGLE_OFFSET);
        sensorHelper.enableContinuousWrap(SlapdownConstants.MIN_ANGLE, SlapdownConstants.MAX_ANGLE);
    }
    public void resetPID(){
        pid.reset();
    }

    @Override
    public void periodic() {
        sensorHelper.update(io.getAngle.getAsDouble());
        fieldsTable.recordOutput("Current command", getCurrentCommand() != null ? getCurrentCommand().getName() : "None");
        //מחובר, אבל הסופליירים encoderלזווית ולהאם ה recordOutput אמור להיות גם pivotלפי ה
        //אמורים לטפל בזה
    }

    public double getCurrent(){
        return io.getCurrent.getAsDouble();
    }
    public boolean isEncoderConnected(){
        return encoderConnectedDebouncer.calculate(io.isEncoderConnected.getAsBoolean());
    }
    public double getAngleDegrees(){
        return sensorHelper.getAngle();
    }
    public double getVelocity(){
        return sensorHelper.getVelocity();
    }

    public void setVoltage(double volt){
        //יש בדיקה של האם הוולט חיובי והאם הזווית מעל המקס (וההפך) אבל כאילו למה שהזווית pivotב
        //תהיה מעל המקס זה לא הכי הגיוני אז לא יודע לא שמתי
        volt = MathUtil.clamp(volt, -SlapdownConstants.MAX_VOLTAGE, SlapdownConstants.MAX_VOLTAGE);
        fieldsTable.recordOutput("Voltage", volt);
    }
    public void stop(){
        io.setVolt(0);
    }
    
    public double calculateFeedforward(double desiredAngle, double desiredSpeed, boolean usePID){
        fieldsTable.recordOutput("desired angle", desiredAngle);
        fieldsTable.recordOutput("desired speed", desiredSpeed);
        double volt = feedforward.calculate(desiredAngle, desiredSpeed);
        return usePID ? volt+pid.calculate(volt) : volt;
        
    }
    public TrapezoidProfile.State calculateTrapezoidProfile(double time, TrapezoidProfile.State initialState, TrapezoidProfile.State desiredState){
        return trapezoidProfile.calculate(time, initialState, desiredState);
    }

    public boolean isAtAngle(double angle){
        return Math.abs(getAngleDegrees() - angle) < SlapdownConstants.ANGLE_TOLLERANCE;
    }
}
