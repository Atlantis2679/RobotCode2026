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
    private SparkMax spindexMotor = new SparkMax(IndexConstants.Canbus.SPINDEX_ID, MotorType.kBrushless);
    private SparkMax indexerMotor = new SparkMax(IndexConstants.Canbus.INDEXER_ID, MotorType.kBrushless);

    private SparkMaxConfig spindexMotorConfig = new SparkMaxConfig();
    private SparkMaxConfig indexerMotorConfig = new SparkMaxConfig();

    public IndexIOSparkMax(LogFieldsTable fields){
        super(fields);

        spindexMotorConfig.smartCurrentLimit(IndexConstants.SPINDEX_CURRENT_LIMIT);
        spindexMotorConfig.idleMode(IdleMode.kCoast);
        REVLibError spinConfigError = spindexMotor.configure(indexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        AlertsFactory.revMotor(PeriodicAlertsGroup.defaultInstance, 
            () -> spinConfigError, spindexMotor::getWarnings, spindexMotor::getFaults, "Spinex Motor Config");

        indexerMotorConfig.smartCurrentLimit(IndexConstants.INDEXER_CURRENT_LIMIT);
        indexerMotorConfig.idleMode(IdleMode.kCoast);
        REVLibError inConfigError = indexerMotor.configure(indexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        AlertsFactory.revMotor(PeriodicAlertsGroup.defaultInstance, 
            () -> inConfigError, indexerMotor::getWarnings, indexerMotor::getFaults, "In Motor Config");
    }

    //Input:
    public void setSpindexVolt(double volt){
        spindexMotor.setVoltage(volt);
    }
    public void setIndexerVolt(double volt){
        indexerMotor.setVoltage(volt);
    }
    
    //Output:
    protected double getSpindexCurrent(){
        return spindexMotor.getOutputCurrent();
    }
    protected double getIndexerCurrent(){
        return indexerMotor.getOutputCurrent();
    }
}

