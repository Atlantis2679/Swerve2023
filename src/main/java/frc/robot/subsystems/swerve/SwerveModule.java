package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.logfields.LogFieldsTable;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.io.SwerveModuleIO;
import frc.robot.subsystems.swerve.io.SwerveModuleIOFalcon;
import frc.robot.subsystems.swerve.io.SwerveModuleIOSim;

import static frc.robot.subsystems.swerve.SwerveContants.*;

public class SwerveModule {

    private final int moduleNumber;

    private final LogFieldsTable fields;
    private final SwerveModuleIO io;

    private final int driveMotorID;
    private final int angleMotorID;
    private final int encoderID;
    private final double angleOffSetDegrees;

    private double lastDriveDistance;
    private double currDriveDistance;

    public SwerveModule(int moduleNumber, int driveMotorID, int angleMotorID, int encoderID,
            double angleOffSetDegrees, LogFieldsTable fieldsTable) {
        this.moduleNumber = moduleNumber;
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.encoderID = encoderID;
        this.angleOffSetDegrees = angleOffSetDegrees;

        fields = fieldsTable.getSubTable("Module " + moduleNumber);
        fields.update();

        io = Robot.isSimulation()
                ? new SwerveModuleIOSim(fields, this.driveMotorID, this.angleMotorID, this.encoderID)
                : new SwerveModuleIOFalcon(fields, this.driveMotorID, this.angleMotorID, this.encoderID);

        lastDriveDistance = io.driveDistanceMeters.getAsDouble();
        currDriveDistance = io.driveDistanceMeters.getAsDouble();
    }

    public void periodic() {
        lastDriveDistance = currDriveDistance;
        currDriveDistance = io.driveDistanceMeters.getAsDouble();
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState = optimize(desiredState, new Rotation2d(Math.toRadians(getIntegratedEncoderAngle())));

        double demandPrcentOutput = desiredState.speedMetersPerSecond / FALCON_MAX_SPEED_MPS;
        io.setDriveSpeed(demandPrcentOutput);

        // only rotate when speed is greater then 1%, to avoid ruining wheels.
        if (Math.abs(desiredState.speedMetersPerSecond) > (FALCON_MAX_SPEED_MPS * 0.01)) {
            io.setAngleMotor(desiredState.angle.getDegrees());
        }
    }

    public void resetToAbsolute() {
        double absoluteAngle = getAbsoluteAngle();

        double absoluteAngleInFalcon = Converstions.degreesToFalcon(absoluteAngle, GEAR_RATIO_DRIVE);

        io.setAngleMotorEncoder(absoluteAngleInFalcon);
    }

    public double getAbsoluteAngle() {
        return io.absoluteAngle.getAsDouble() - angleOffSetDegrees;
    }

    public int getModuleNumber() {
        return this.moduleNumber;
    }

    public double getDistanceMeters() {
        return io.driveDistanceMeters.getAsDouble();
    }

    public double getIntegratedEncoderAngle() {
        return io.integratedEncoderAngle.getAsDouble();
    }

    public double placeInAppropriateScope(double currentAngle, double targetAngle) {
        int scope = (int) currentAngle / 360;

        double lowerBound = currentAngle >= 0 ? scope * 360 : (scope - 1) * 360;
        double upperBound = currentAngle >= 0 ? (scope + 1) * 360 : scope * 360;

        while (targetAngle < lowerBound) {
            targetAngle += 360;
        }
        while (targetAngle > upperBound) {
            targetAngle -= 360;
        }

        if (targetAngle - currentAngle > 180) {
            targetAngle -= 360;
        } else if (targetAngle - currentAngle < -180) {
            targetAngle += 360;
        }

        return targetAngle;
    }

    public SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        double targetAngle = placeInAppropriateScope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
        double targetSpeed = desiredState.speedMetersPerSecond;

        double delta = targetAngle - currentAngle.getDegrees();

        if (Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;
            targetAngle = delta > 0 ? (targetAngle - 180) : (targetAngle + 180);
        }

        return new SwerveModuleState(targetSpeed, new Rotation2d(Math.toRadians(targetAngle)));
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getModuleMPS(), getRotation2d());
    }

    public double getModuleMPS() {
        return io.driveSpeedMPS.getAsDouble();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(io.absoluteAngle.getAsDouble());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(io.driveDistanceMeters.getAsDouble(), getRotation2d());
    }

    public SwerveModulePosition getModulePositionDelta() {
        return new SwerveModulePosition(
                currDriveDistance - lastDriveDistance,
                getRotation2d());
    }
}
