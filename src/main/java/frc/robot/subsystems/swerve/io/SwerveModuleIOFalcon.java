package frc.robot.subsystems.swerve.io;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import frc.robot.subsystems.swerve.Converstions;
import frc.robot.subsystems.swerve.SwerveContants;
import frc.lib.logfields.LogFieldsTable;

public class SwerveModuleIOFalcon extends SwerveModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX angleMotor;
    private final CANCoder canCoder;

    public SwerveModuleIOFalcon(LogFieldsTable fieldsTable, int driveMotorID, int angleMotorID, int encoderID) {
        super(fieldsTable);

        driveMotor = new TalonFX(driveMotorID);
        angleMotor = new TalonFX(angleMotorID);
        canCoder = new CANCoder(encoderID);

        TalonFXConfiguration driveMotorConfiguration = new TalonFXConfiguration();
        TalonFXConfiguration angleMotorConfiguration = new TalonFXConfiguration();
        CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();

        driveMotor.configAllSettings(driveMotorConfiguration);
        angleMotor.configAllSettings(angleMotorConfiguration);
        canCoder.configAllSettings(canCoderConfiguration);
    }

    @Override 
    protected double getAbsoluteAngleDegrees() {
        return canCoder.getAbsolutePosition();
    }

    @Override
    protected double getDriveSpeedMPS() {
        return Converstions.falconToMPS(driveMotor.getSelectedSensorPosition(), SwerveContants.WHEEL_CIRCUMFERENCE_M, SwerveContants.GEAR_RATIO_DRIVE);
    }

    @Override
    protected double getIntegratedEncoderDegrees() {
        return Converstions.falconToDegrees(angleMotor.getSelectedSensorPosition(), SwerveContants.GEAR_RATIO_ANGLE);
    }

    @Override
    protected double getDriveDistanceMeters() {
        return Converstions.falconToMeters(driveMotor.getSelectedSensorPosition(), SwerveContants.WHEEL_CIRCUMFERENCE_M, SwerveContants.GEAR_RATIO_DRIVE);
    }

    @Override
    public void setDriveSpeed(double demandPrcentOutput) {
        driveMotor.set(ControlMode.PercentOutput, demandPrcentOutput);
    }

    @Override
    public void setAngleMotor(double degrees) {
        double angleTics = Converstions.degreesToFalcon(degrees, SwerveContants.GEAR_RATIO_ANGLE);
        angleMotor.set(ControlMode.Position, angleTics);
    }

    @Override
    public void setAngleMotorEncoder(double angleDegrees) {
        angleMotor.setSelectedSensorPosition(angleDegrees);
    }
}