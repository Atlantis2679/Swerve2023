package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.logfields.LogFieldsTable;
import frc.lib.tuneables.Tuneable;
import frc.lib.tuneables.TuneableBuilder;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.io.SwerveModuleIO;
import frc.robot.subsystems.swerve.io.SwerveModuleIOFalcon;
import frc.robot.subsystems.swerve.io.SwerveModuleIOSim;

import static frc.robot.subsystems.swerve.SwerveContants.*;

public class SwerveModule implements Tuneable {
    private final int moduleNumber;

    private final LogFieldsTable fieldsTable;
    private final SwerveModuleIO io;

    private final int driveMotorID;
    private final int angleMotorID;
    private final int encoderID;
    private double angleOffSetDegrees;

    private double lastDriveDistanceMeters;
    private double currDriveDistanceMeters;
    private boolean encoderResetToAbsoluteQueued = false;

    private final double WHEEL_CIRCUMFERENCE_METERS = 2 * Math.PI * WHEEL_RADIUS_METERS;

    public SwerveModule(int moduleNumber, int driveMotorID, int angleMotorID, int encoderID,
            double angleOffSetDegrees, LogFieldsTable swerveFieldsTable) {
        this.moduleNumber = moduleNumber;
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.encoderID = encoderID;
        this.angleOffSetDegrees = angleOffSetDegrees;

        fieldsTable = swerveFieldsTable.getSubTable("Module " + moduleNumber);
        fieldsTable.update();

        io = Robot.isSimulation()
                ? new SwerveModuleIOSim(fieldsTable, this.driveMotorID, this.angleMotorID, this.encoderID)
                : new SwerveModuleIOFalcon(fieldsTable, this.driveMotorID, this.angleMotorID, this.encoderID);

        lastDriveDistanceMeters = getDriveDistanceMeters();
        currDriveDistanceMeters = getDriveDistanceMeters();

        io.setIntegratedEncoderAngleEncoderRotations(getAbsoluteAngleDegrees() / 360);
    }

    public void periodic() {
        lastDriveDistanceMeters = currDriveDistanceMeters;
        currDriveDistanceMeters = getDriveDistanceMeters();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean preventJittering) {
        if (preventJittering && Math.abs(desiredState.speedMetersPerSecond) < FALCON_MAX_SPEED_MPS * 0.01) {
            io.setDriveSpeedPrecentage(0);
            return;
        }

        final double currentAngleDegrees;

        if (encoderResetToAbsoluteQueued) {
            io.setIntegratedEncoderAngleEncoderRotations(getAbsoluteAngleDegrees() / 360);
            currentAngleDegrees = getAbsoluteAngleDegrees();
            encoderResetToAbsoluteQueued = false;
        } else {
            currentAngleDegrees = getIntegratedEncoderAngleDegrees();
        }

        SwerveModuleState optimizedDesiredState = optimize(desiredState, currentAngleDegrees);

        io.setDriveSpeedPrecentage(optimizedDesiredState.speedMetersPerSecond / FALCON_MAX_SPEED_MPS);
        io.setAngleMotorPositionRotations(optimizedDesiredState.angle.getRotations());
    }

    public void queueResetToAbsolute() {
        encoderResetToAbsoluteQueued = true;
    }

    public double getAbsoluteAngleDegrees() {
        return (io.absoluteAngleRotations.getAsDouble() * 360) - angleOffSetDegrees;
    }

    public int getModuleNumber() {
        return this.moduleNumber;
    }

    public double getDriveDistanceMeters() {
        return io.driveMotorRotations.getAsDouble() * WHEEL_CIRCUMFERENCE_METERS;
    }

    public double getIntegratedEncoderAngleDegrees() {
        return io.integratedEncoderAngleRotations.getAsDouble() * 360;
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getModuleMPS(), getRotation2d());
    }

    public double getModuleMPS() {
        return io.driveSpeedRPS.getAsDouble() * WHEEL_CIRCUMFERENCE_METERS;
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getAbsoluteAngleDegrees());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(currDriveDistanceMeters, getRotation2d());
    }

    public SwerveModulePosition getModulePositionDelta() {
        return new SwerveModulePosition(
                currDriveDistanceMeters - lastDriveDistanceMeters,
                getRotation2d());
    }

    public void setAbsoluteEncoderAngleDegrees(double degrees) {
        angleOffSetDegrees = (io.absoluteAngleRotations.getAsDouble() * 360) - degrees;
        queueResetToAbsolute();
    }

    public double getP() {
        return io.kP.getAsDouble();
    }

    public double getI() {
        return io.kI.getAsDouble();
    }

    public double getD() {
        return io.kD.getAsDouble();
    }

    public void setP(double p) {
        io.setP(p);
    }

    public void setI(double i) {
        io.setI(i);
    }

    public void setD(double d) {
        io.setD(d);
    }

    private SwerveModuleState optimize(SwerveModuleState desiredState, double currentAngleDegrees) {
        double targetAngleDegrees = placeInAppropriateScope(currentAngleDegrees, desiredState.angle.getDegrees());
        double targetSpeedMPS = desiredState.speedMetersPerSecond;

        double delta = targetAngleDegrees - currentAngleDegrees;

        if (Math.abs(delta) > 90) {
            targetSpeedMPS = -targetSpeedMPS;
            targetAngleDegrees = delta > 0 ? (targetAngleDegrees - 180) : (targetAngleDegrees + 180);
        }

        return new SwerveModuleState(targetSpeedMPS, Rotation2d.fromDegrees(targetAngleDegrees));
    }

    private double placeInAppropriateScope(double currentAngleDegrees, double targetAngleDegrees) {
        int scope = (int) currentAngleDegrees / 360;

        double lowerBound = currentAngleDegrees >= 0 ? scope * 360 : (scope - 1) * 360;
        double upperBound = currentAngleDegrees >= 0 ? (scope + 1) * 360 : scope * 360;

        while (targetAngleDegrees < lowerBound) {
            targetAngleDegrees += 360;
        }
        while (targetAngleDegrees > upperBound) {
            targetAngleDegrees -= 360;
        }

        if (targetAngleDegrees - currentAngleDegrees > 180) {
            targetAngleDegrees -= 360;
        } else if (targetAngleDegrees - currentAngleDegrees < -180) {
            targetAngleDegrees += 360;
        }

        return targetAngleDegrees;
    }

    @Override
    public void initTuneable(TuneableBuilder builder) {
        builder.addDoubleProperty("Integrated Angle Degrees", this::getIntegratedEncoderAngleDegrees, null);
        builder.addDoubleProperty("Absolute Angle Degrees", this::getAbsoluteAngleDegrees, null);
        builder.addDoubleProperty("Tuneable Offset",
                () -> angleOffSetDegrees,
                val -> {
                    angleOffSetDegrees = val;
                    queueResetToAbsolute();
                });
    }
}
