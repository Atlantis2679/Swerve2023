package frc.robot.subsystems.swerve.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

import frc.lib.logfields.LogFieldsTable;

public class SwerveModuleIOFalcon extends SwerveModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX angleMotor;
    private final CANcoder canCoder;

    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

    public SwerveModuleIOFalcon(LogFieldsTable fieldsTable, int driveMotorID, int angleMotorID, int encoderID) {
        super(fieldsTable);

        driveMotor = new TalonFX(driveMotorID);
        angleMotor = new TalonFX(angleMotorID);
        canCoder = new CANcoder(encoderID);

        TalonFXConfiguration driveMotorConfiguration = new TalonFXConfiguration();
        TalonFXConfiguration angleMotorConfiguration = new TalonFXConfiguration();
        CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();

        driveMotor.getConfigurator().apply(driveMotorConfiguration);
        angleMotor.getConfigurator().apply(angleMotorConfiguration);
        canCoder.getConfigurator().apply(canCoderConfiguration);

        
    }

    @Override 
    protected double getAbsoluteAngle() {
        return canCoder.getAbsolutePosition().getValueAsDouble();
    }

    @Override
    protected double getDriveSpeed() {
        return driveMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void setDriveSpeed(double demandPrcentOutput) {
        driveMotor.set(demandPrcentOutput);
    }

    @Override
    public void setAngleMotor(double angleTics) {
        angleMotor.setControl(dutyCycleOut.withOutput(angleTics));
    }

    @Override
    public void setAngleMotorEncoder(double angleDegrees) {
        angleMotor.setPosition(angleDegrees);
    }
}