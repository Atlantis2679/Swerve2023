package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.swerve.io.SwerveModuleIO;
import frc.robot.subsystems.swerve.io.SwerveModuleIOFalcon;
import frc.robot.utils.fields.FieldsTable;

public class SwerveModule {
    private final int moduleNumber;
    
    private final FieldsTable fields;
    private final SwerveModuleIO io;

    private final int driveMotorID;
    private final int angleMotorID;
    private final int angleEncoderID;
    private final Rotation2d angleOffSet;

    public SwerveModule(int moduleNumber, int driveMotorID, int angleMotorID, int angleEncoderID, double angleOffSet) {

        this.moduleNumber = moduleNumber;
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.angleEncoderID = angleEncoderID;
        this.angleOffSet = new Rotation2d(Math.toRadians(angleOffSet));

        fields = new FieldsTable("Swerve Module " + moduleNumber);
        io = new SwerveModuleIOFalcon(fields, this.driveMotorID, this.angleMotorID, this.angleEncoderID);
    }


    public void SetDesiredState(SwerveModuleState moduleState) {
        double demand = moduleState.speedMetersPerSecond / SwerveContants.FALCON_MAX_SPEED;
        io.setDriveSpeed(demand);

        double angle = Utils.degreesToFalcon(moduleState.angle.getDegrees(), SwerveContants.GEAR_RATIO);
        io.setAngleMotor(angle);
    }

    
    public void resetToAbsolute() {
        double currentDegreesCAN = io.absoluteAngle.get();
        double absoluteAngle = currentDegreesCAN - angleOffSet.getDegrees();

        double absoluteFalcon = absoluteAngle * (360.0 / (SwerveContants.GEAR_RATIO * 2048));

        io.setAngleMotorEncoder(absoluteFalcon);
    }


    public int getModuleNumber() { 
        return this.moduleNumber;
    }


    public double getMeters() {
        return Utils.falconToMeters(io.driveSpeed.get(), SwerveContants.CIRCUMFERENCE, SwerveContants.GEAR_RATIO);
    }


}
