package frc.robot.subsystems.swerve.io;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import frc.robot.utils.fields.FieldsTable;

public class SwerveModuleIOFalcon extends SwerveModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX angleMotor;
    private final CANCoder canCoder;

    public SwerveModuleIOFalcon(FieldsTable fieldsTable, int driveMotorID, int angleMotorID, int encoderID) {
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
    protected double getAbsoluteAngle() {
        return canCoder.getAbsolutePosition();
    }

    @Override
    protected double getDriveSpeed() {
        return driveMotor.getSelectedSensorPosition();
    }

    @Override
    protected double getIntegratedAngle() {
        return angleMotor.getSelectedSensorPosition();
    }

    @Override
    public void setDriveSpeed(double demandPrcentOutput) {
        driveMotor.set(ControlMode.PercentOutput, demandPrcentOutput);
    }

    @Override
    public void setAngleMotor(double angleTics) {
        angleMotor.set(ControlMode.Position, angleTics);
    }

    @Override
    public void setAngleMotorEncoder(double angleDegrees) {
        angleMotor.setSelectedSensorPosition(angleDegrees);
    }
}