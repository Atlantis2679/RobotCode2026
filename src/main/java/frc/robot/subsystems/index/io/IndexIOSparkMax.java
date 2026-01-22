package frc.robot.subsystems.index.io;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.RobotMap.CANBUS;
import frc.robot.subsystems.index.IndexConstants;
import frc.robot.utils.AlertsFactory;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.periodicalerts.PeriodicAlertsGroup;

public class IndexIOSparkMax extends IndexIO {
    private SparkMax spindexMotor = new SparkMax(CANBUS.SPINDEX_ID, MotorType.kBrushless);
    private SparkMax indexerMotor = new SparkMax(CANBUS.INDEXER_ID, MotorType.kBrushless);

    public IndexIOSparkMax(LogFieldsTable fields){
        super(fields);

        PeriodicAlertsGroup alerts = new PeriodicAlertsGroup("Index");
        
        SparkMaxConfig spindexMotorConfig = new SparkMaxConfig();
        spindexMotorConfig.smartCurrentLimit(IndexConstants.SPINDEX_CURRENT_LIMIT);
        spindexMotorConfig.idleMode(IdleMode.kCoast);
        REVLibError spindexConfigError = spindexMotor.configure(spindexMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        AlertsFactory.revMotor(alerts, 
            () -> spindexConfigError, spindexMotor::getWarnings, spindexMotor::getFaults, "Spinex Motor");

        SparkMaxConfig indexerMotorConfig = new SparkMaxConfig();
        indexerMotorConfig.smartCurrentLimit(IndexConstants.INDEXER_CURRENT_LIMIT);
        indexerMotorConfig.idleMode(IdleMode.kCoast);
        REVLibError indexerConfigError = indexerMotor.configure(indexerMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        AlertsFactory.revMotor(alerts, () -> indexerConfigError, indexerMotor::getWarnings, indexerMotor::getFaults, "Indexer Motor");
    }

    public void setSpindexVolt(double volt){
        spindexMotor.setVoltage(volt);
    }

    public void setIndexerVolt(double volt){
        indexerMotor.setVoltage(volt);
    }
    
    protected double getSpindexCurrent(){
        return spindexMotor.getOutputCurrent();
    }
    
    protected double getIndexerCurrent(){
        return indexerMotor.getOutputCurrent();
    }
}

