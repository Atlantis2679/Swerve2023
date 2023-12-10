package frc.robot.subsystems.swerve.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;

import static frc.robot.subsystems.swerve.SwerveContants.*;

import frc.lib.logfields.LogFieldsTable;

public class SwerveModuleIOFalcon extends SwerveModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX angleMotor;
    private final CANcoder canCoder;

    private final PositionDutyCycle positionDutyCycleControl = new PositionDutyCycle(0);
    private final Slot0Configs slot0Configs = new Slot0Configs();

    public SwerveModuleIOFalcon(LogFieldsTable fieldsTable, int driveMotorID, int angleMotorID, int encoderID) {
        super(fieldsTable);

        driveMotor = new TalonFX(driveMotorID);
        angleMotor = new TalonFX(angleMotorID);
        canCoder = new CANcoder(encoderID);

        TalonFXConfiguration driveMotorConfiguration = new TalonFXConfiguration();
        TalonFXConfiguration angleMotorConfiguration = new TalonFXConfiguration();
        slot0Configs.kP = KP;
        slot0Configs.kI = KI;
        slot0Configs.kD = KD;
        driveMotor.getConfigurator().apply(slot0Configs);
        positionDutyCycleControl.Slot = 0;
        CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();

        driveMotor.getConfigurator().apply(driveMotorConfiguration);
        angleMotor.getConfigurator().apply(angleMotorConfiguration);
        canCoder.getConfigurator().apply(canCoderConfiguration);
    }

    @Override
    protected double getAbsoluteAngleRotations() {
        return canCoder.getAbsolutePosition().getValueAsDouble();
    }

    @Override
    protected double getDriveSpeedRPS() {
        return driveMotor.getRotorVelocity().getValueAsDouble() / GEAR_RATIO_DRIVE;
    }

    @Override
    protected double getDriveMotorRotations() {
        return driveMotor.getRotorPosition().getValueAsDouble() / GEAR_RATIO_DRIVE;
    }

    @Override
    protected double getIntegratedAngleEncoderRotations() {
        return angleMotor.getRotorPosition().getValueAsDouble() / GEAR_RATIO_ANGLE;
    }

    @Override
    protected double getP() {
        return slot0Configs.kP;
    }

    @Override
    protected double getI() {
        return slot0Configs.kI;
    }

    @Override
    protected double getD() {
        return slot0Configs.kD;
    }

    @Override
    public void setDriveSpeedPrecentage(double demandPrcentOutput) {
        driveMotor.set(demandPrcentOutput);
    }

    @Override
    public void setAngleMotorRotations(double rotations) {
        angleMotor.setControl(positionDutyCycleControl.withPosition(rotations / GEAR_RATIO_ANGLE));
    }

    @Override
    public void setIntegratedAngleEncoderRotations(double angleRotations) {
        angleMotor.setPosition(angleRotations / GEAR_RATIO_ANGLE);
    }

    @Override
    public void setP(double p) {
        slot0Configs.kP = p;
    }

    @Override
    public void setI(double i) {
        slot0Configs.kI = i;
    }

    @Override
    public void setD(double d) {
        slot0Configs.kD = d;
    }
}