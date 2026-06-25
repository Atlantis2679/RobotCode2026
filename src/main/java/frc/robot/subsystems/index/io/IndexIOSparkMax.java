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
    private SparkMax spindexLeaderMotor = new SparkMax(CANBUS.SPINDEX_LEADER_ID, MotorType.kBrushless);
    private SparkMax spindexFollowerMotor = new SparkMax(CANBUS.SPINDEX_FOLLOWER_ID, MotorType.kBrushless);
    private SparkMax indexerMotor = new SparkMax(CANBUS.INDEXER_ID, MotorType.kBrushless);

    public IndexIOSparkMax(LogFieldsTable fields){
        super(fields);

        PeriodicAlertsGroup alerts = PeriodicAlertsGroup.defaultInstance.getSubGroup("Index");
        
        SparkMaxConfig spindexMotorConfig = new SparkMaxConfig();
        spindexMotorConfig.smartCurrentLimit(IndexConstants.SPINDEX_LEARDER_CURRENT_LIMIT);
        spindexMotorConfig.idleMode(IdleMode.kCoast);
        spindexMotorConfig.inverted(true);
        REVLibError spindexLeaderConfigError = spindexLeaderMotor.configure(spindexMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        AlertsFactory.revMotor(alerts, 
            () -> spindexLeaderConfigError, spindexLeaderMotor::getWarnings, spindexLeaderMotor::getFaults, "Spinex Leader Motor");
        spindexMotorConfig.follow(CANBUS.SPINDEX_LEADER_ID);
        spindexMotorConfig.smartCurrentLimit(IndexConstants.SPINDEX_FOLLOWER_CURRENT_LIMIT);
        REVLibError spindexFollowerConfigError = spindexFollowerMotor.configure(spindexMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        AlertsFactory.revMotor(alerts, 
            () -> spindexFollowerConfigError, spindexFollowerMotor::getWarnings, spindexFollowerMotor::getFaults, "Spinex Follower Motor");
        
        SparkMaxConfig indexerMotorConfig = new SparkMaxConfig();
        indexerMotorConfig.smartCurrentLimit(IndexConstants.INDEXER_CURRENT_LIMIT);
        indexerMotorConfig.idleMode(IdleMode.kCoast);
        indexerMotorConfig.inverted(true);
        REVLibError indexerConfigError = indexerMotor.configure(indexerMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        AlertsFactory.revMotor(alerts, () -> indexerConfigError, indexerMotor::getWarnings, indexerMotor::getFaults, "Indexer Motor");
    }

    public void setSpindexVolt(double volt){
        spindexLeaderMotor.setVoltage(volt);
    }

    public void setIndexerVolt(double volt){
        indexerMotor.setVoltage(volt);
    }
    
    protected double getSpindexLeaderCurrent(){
        return spindexLeaderMotor.getOutputCurrent();
    }

    protected double getSpindexFollowerCurrent(){
        return spindexFollowerMotor.getOutputCurrent();
    }
    
    protected double getIndexerCurrent(){
        return indexerMotor.getOutputCurrent();
    }
}

