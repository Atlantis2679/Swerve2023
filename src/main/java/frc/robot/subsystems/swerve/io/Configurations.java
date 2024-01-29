package frc.robot.subsystems.swerve.io;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static frc.robot.subsystems.swerve.SwerveContants.*;

public class Configurations {
    public static TalonFXConfiguration getDriveMotorConfigs() {
        TalonFXConfiguration driveMotorConfiguration = new TalonFXConfiguration();
        driveMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveMotorConfiguration.CurrentLimits.SupplyCurrentLimit = 35;
        driveMotorConfiguration.CurrentLimits.SupplyCurrentThreshold = 60;
        driveMotorConfiguration.CurrentLimits.SupplyTimeThreshold = 0.1;

        return driveMotorConfiguration;
    }

    public static TalonFXConfiguration getAngleMotorConfiguration() {
        TalonFXConfiguration angleMotorConfiguration = new TalonFXConfiguration();

        angleMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        angleMotorConfiguration.Feedback.SensorToMechanismRatio = GEAR_RATIO_ANGLE;
        angleMotorConfiguration.ClosedLoopGeneral.ContinuousWrap = true;
        angleMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        angleMotorConfiguration.CurrentLimits.SupplyCurrentLimit = 25;
        angleMotorConfiguration.CurrentLimits.SupplyCurrentThreshold = 40;
        angleMotorConfiguration.CurrentLimits.SupplyTimeThreshold = 0.1;

        return angleMotorConfiguration;
    }
}
