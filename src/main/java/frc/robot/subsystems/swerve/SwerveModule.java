package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.logfields.LogFieldsTable;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.io.SwerveModuleIO;
import frc.robot.subsystems.swerve.io.SwerveModuleIOFalcon;
import frc.robot.subsystems.swerve.io.SwerveModuleIOSim;

public class SwerveModule {

    private final int moduleNumber;

    private final LogFieldsTable fields;
    private final SwerveModuleIO io;

    private final int driveMotorID;
    private final int angleMotorID;
    private final int encoderID;
    private final Rotation2d angleOffSet;

    public SwerveModule(int moduleNumber, int driveMotorID, int angleMotorID, int encoderID,
        double angleOffSetDegrees, LogFieldsTable fieldsTable) {

        fields = fieldsTable.getSubTable("Module " + getModuleNumber());
        fields.update();

        this.moduleNumber = moduleNumber;
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.encoderID = encoderID;
        this.angleOffSet = new Rotation2d(Math.toRadians(angleOffSetDegrees));

        io = Robot.isSimulation()
            ? new SwerveModuleIOSim(fields, this.driveMotorID, this.angleMotorID, this.encoderID)
            : new SwerveModuleIOFalcon(fields, this.driveMotorID, this.angleMotorID, this.encoderID);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        fields.recordOutput("integrated angle", getIntegratedEncoderAngle());
        desiredState = optimize(desiredState, new Rotation2d(Math.toRadians(getIntegratedEncoderAngle())));

        double demandPrcentOutput = desiredState.speedMetersPerSecond / SwerveContants.FALCON_MAX_SPEED_MPS;
        io.setDriveSpeed(demandPrcentOutput);

        if (Math.abs(desiredState.speedMetersPerSecond) > (SwerveContants.FALCON_MAX_SPEED_MPS * 0.01)) {
            io.setAngleMotor(desiredState.angle.getDegrees());
        }
    }

    public void resetToAbsolute() {
        double absoluteAngle = getAbsoluteAngle();

        double absoluteAngleInFalcon = Converstions.degreesToFalcon(absoluteAngle, SwerveContants.GEAR_RATIO_DRIVE);

        io.setAngleMotorEncoder(absoluteAngleInFalcon);
    }

    public double getAbsoluteAngle() {
        return io.absoluteAngle.getAsDouble() - angleOffSet.getDegrees();
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

        fields.recordOutput("speed in optimize", targetSpeed);
        fields.recordOutput("angle after optimize", targetAngle);

        return new SwerveModuleState(targetSpeed, new Rotation2d(Math.toRadians(targetAngle)));
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getModuleMPS(), new Rotation2d(Math.toRadians(io.absoluteAngle.getAsDouble())));
    }

    public double getModuleMPS() {
        return io.driveSpeedMPS.getAsDouble();
    }

}
