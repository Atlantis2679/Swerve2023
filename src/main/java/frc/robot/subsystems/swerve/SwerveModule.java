package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.RobotMap;
import frc.robot.subsystems.swerve.io.SwerveModuleIO;
import frc.robot.subsystems.swerve.io.SwerveModuleIOFalcon;
import frc.robot.utils.fields.FieldsTable;

public class SwerveModule {
    private final int moduleNumber;

    private final FieldsTable fields;
    private final SwerveModuleIO io;

    private final int driveMotorID;
    private final int angleMotorID;
    private final int encoderID;
    private final Rotation2d angleOffSet;

    public SwerveModule(int moduleNumber, int driveMotorID, int angleMotorID, int encoderID,
            double angleOffSetDegrees) {

        this.moduleNumber = moduleNumber;
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.encoderID = encoderID;
        this.angleOffSet = new Rotation2d(Math.toRadians(angleOffSetDegrees));

        fields = new FieldsTable("Swerve Module " + this.moduleNumber);
        io = new SwerveModuleIOFalcon(fields, this.driveMotorID, this.angleMotorID, this.encoderID);
    }

    public void SetDesiredState(SwerveModuleState moduleState) {
        double demandPrcentOutput = moduleState.speedMetersPerSecond / RobotMap.FALCON_MAX_SPEED;
        io.setDriveSpeed(demandPrcentOutput);

        double angleTics = Converstions.degreesToFalcon(moduleState.angle.getDegrees(), RobotMap.GEAR_RATIO);
        io.setAngleMotor(angleTics);
    }

    public void resetToAbsolute() {
        double absoluteAngle = getAbsoluteAngle();
        
        double absoluteFalcon = Converstions.degreesToFalcon(absoluteAngle, RobotMap.GEAR_RATIO);

        io.setAngleMotorEncoder(absoluteFalcon);
    }

    public double getAbsoluteAngle() {
        return io.absoluteAngle.get() - angleOffSet.getDegrees();
    }

    public int getModuleNumber() {
        return this.moduleNumber;
    }

    public double getMeters() {
        return Converstions.falconToMeters(io.driveSpeed.get(), RobotMap.WHEEL_CIRCUMFERENCE, RobotMap.GEAR_RATIO);
    }

}
