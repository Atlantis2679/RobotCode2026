package frc.robot.subsystems.index.io;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.subsystems.index.IndexConstants;
import frc.robot.utils.AlertsFactory;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.periodicalerts.PeriodicAlertsGroup;

public class IndexIOSparkMax extends IndexIO {
    private SparkMax spinMotor = new SparkMax(IndexConstants.Canbus.SPINDEX_ID, MotorType.kBrushless);
    private SparkMax inMotor = new SparkMax(IndexConstants.Canbus.INDEXER_ID, MotorType.kBrushless);

    private SparkMaxConfig spinMotorConfig = new SparkMaxConfig();
    private SparkMaxConfig inMotorConfig = new SparkMaxConfig();

    public IndexIOSparkMax(LogFieldsTable fields){
        super(fields);

        spinMotorConfig.smartCurrentLimit(IndexConstants.SpinCurrentLimit);
        spinMotorConfig.idleMode(IdleMode.kCoast);
        REVLibError spinConfigError = spinMotor.configure(inMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        AlertsFactory.revMotor(PeriodicAlertsGroup.defaultInstance, 
            () -> spinConfigError, spinMotor::getWarnings, spinMotor::getFaults, "Spinex Motor Config");

        inMotorConfig.smartCurrentLimit(IndexConstants.InCurrentLimit);
        inMotorConfig.idleMode(IdleMode.kCoast);
        REVLibError inConfigError = inMotor.configure(inMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        AlertsFactory.revMotor(PeriodicAlertsGroup.defaultInstance, 
            () -> inConfigError, inMotor::getWarnings, inMotor::getFaults, "In Motor Config");
    }

    //Input:
    public void setSpindexVolt(double volt){
        spinMotor.setVoltage(volt);
    }
    public void setIndexerVolt(double volt){
        inMotor.setVoltage(volt);
    }
    
    //Output:
    protected double getSpindexCurrent(){
        return spinMotor.getOutputCurrent();
    }
    protected double getIndexerCurrent(){
        return inMotor.getOutputCurrent();
    }
}

