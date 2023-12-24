package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.io.GyroIO;
import frc.robot.subsystems.swerve.io.GyroIONavX;
import frc.robot.subsystems.swerve.io.GyroIOSim;
import frc.lib.logfields.LogFieldsTable;
import frc.lib.tuneables.SendableType;
import frc.lib.tuneables.Tuneable;
import frc.lib.tuneables.TuneablesManager;
import frc.lib.tuneables.TuneablesTable;
import frc.lib.valueholders.DoubleHolder;
import frc.robot.Robot;
import frc.robot.RobotMap.Module0;
import frc.robot.RobotMap.Module1;
import frc.robot.RobotMap.Module2;
import frc.robot.RobotMap.Module3;

import static frc.robot.subsystems.swerve.SwerveContants.*;

public class Swerve extends SubsystemBase {
    private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());
    private final TuneablesTable tuneablesTable = new TuneablesTable(SendableType.LIST);

    private final GyroIO gyroIO = Robot.isSimulation()
            ? new GyroIOSim(fieldsTable.getSubTable("Gyro"))
            : new GyroIONavX(fieldsTable.getSubTable("Gyro"));

    private final SwerveDriveOdometry odometry;

    private final SwerveModule[] modules = {
            new SwerveModule(0, Module0.DRIVE_MOTOR_ID, Module0.ANGLE_MOTOR_ID, Module0.ENCODER_ID,
                    MODULE_0_ANGLE_OFFSET_DEGREES, fieldsTable),
            new SwerveModule(1, Module1.DRIVE_MOTOR_ID, Module1.ANGLE_MOTOR_ID, Module1.ENCODER_ID,
                    MODULE_1_ANGLE_OFFSET_DEGREES, fieldsTable),
            new SwerveModule(2, Module2.DRIVE_MOTOR_ID, Module2.ANGLE_MOTOR_ID, Module2.ENCODER_ID,
                    MODULE_2_ANGLE_OFFSET_DEGREES, fieldsTable),
            new SwerveModule(3, Module3.DRIVE_MOTOR_ID, Module3.ANGLE_MOTOR_ID, Module3.ENCODER_ID,
                    MODULE_3_ANGLE_OFFSET_DEGREES, fieldsTable) };

    // The x and y might seem a bit weird, but this is how they are defined in
    // WPILib. For more info:
    // https://docs.wpilib.org/he/stable/docs/software/advanced-controls/geometry/coordinate-systems.html
    public final Translation2d FRONT_LEFT_LOCATION = new Translation2d(
            SwerveContants.TRACK_LENGTH_M / 2,
            SwerveContants.TRACK_WIDTH_M / 2);
    public final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(
            SwerveContants.TRACK_LENGTH_M / 2,
            -SwerveContants.TRACK_WIDTH_M / 2);
    public final Translation2d BACK_LEFT_LOCATION = new Translation2d(
            SwerveContants.TRACK_WIDTH_M / 2,
            -SwerveContants.TRACK_LENGTH_M / 2);
    public final Translation2d BACK_RIGHT_LOCATION = new Translation2d(
            -SwerveContants.TRACK_WIDTH_M / 2,
            -SwerveContants.TRACK_LENGTH_M / 2);

    private final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            FRONT_LEFT_LOCATION,
            FRONT_RIGHT_LOCATION,
            BACK_LEFT_LOCATION,
            BACK_RIGHT_LOCATION);

    private double lastYaw = 0;

    public Swerve() {
        fieldsTable.update();

        odometry = new SwerveDriveOdometry(
                swerveKinematics,
                gyroIO.isConnected.getAsBoolean() ? getRotation2d() : new Rotation2d(),
                getModulesPositions());

        tuneablesTable.addChild("Modules Angle PID", (Tuneable) (builderPID) -> {
            builderPID.setSendableType(SendableType.PID);
            builderPID.addDoubleProperty("p", modules[0]::getP, (p) -> {
                for (SwerveModule module : modules) {
                    module.setP(p);
                }
            });

            builderPID.addDoubleProperty("i", modules[0]::getI, (i) -> {
                for (SwerveModule module : modules) {
                    module.setI(i);
                }
            });

            builderPID.addDoubleProperty("d", modules[0]::getD, (d) -> {
                for (SwerveModule module : modules) {
                    module.setD(d);
                }
            });
            builderPID.addDoubleProperty("setpoint", () -> 0, null);
        });
        tuneablesTable.addChild("Module 0", modules[0]);
        tuneablesTable.addChild("Module 1", modules[1]);
        tuneablesTable.addChild("Module 2", modules[2]);
        tuneablesTable.addChild("Module 3", modules[3]);

        tuneablesTable.addChild("reset modules absolute position", (Tuneable) (builder) -> {
            DoubleHolder angleToResetDegrees = new DoubleHolder(0);
            builder.addDoubleProperty("angle to reset degrees", angleToResetDegrees::get, angleToResetDegrees::set);
            builder.addChild("reset!", new InstantCommand(() -> {
                for (SwerveModule swerveModule : modules) {
                    swerveModule.setAbsoluteEncoderAngleDegrees(angleToResetDegrees.get());
                }
            }));
        });

        tuneablesTable.addChild("reset to absolute", new InstantCommand(this::requestResetModulesToAbsolute));

        TuneablesManager.add("Swerve", tuneablesTable);
    }

    @Override
    public void periodic() {
        for (SwerveModule module : modules) {
            module.periodic();
        }

        if (gyroIO.isConnected.getAsBoolean()) {
            odometry.update(Rotation2d.fromDegrees(gyroIO.yaw.getAsDouble()), getModulesPositions());
            lastYaw = gyroIO.yaw.getAsDouble();
        } else {
            Twist2d twist = swerveKinematics.toTwist2d(
                    modules[0].getModulePositionDelta(),
                    modules[1].getModulePositionDelta(),
                    modules[2].getModulePositionDelta(),
                    modules[3].getModulePositionDelta());

            odometry.update(Rotation2d.fromRadians(Math.toRadians(lastYaw) + twist.dtheta), getModulesPositions());
            lastYaw += Math.toDegrees(twist.dtheta);
        }

        fieldsTable.recordOutput("Odometry", odometry.getPoseMeters());
        fieldsTable.recordOutput("Module States",
                modules[0].getModuleState(),
                modules[1].getModuleState(),
                modules[2].getModuleState(),
                modules[3].getModuleState());
    }

    public void drive(Translation2d translation, double angularVelocity, boolean isFieldRelative) {
        ChassisSpeeds desiredChassisSpeeds;

        if (isFieldRelative) {
            desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(),
                    translation.getY(),
                    angularVelocity,
                    getRotation2d());
        } else {
            desiredChassisSpeeds = new ChassisSpeeds(
                    translation.getX(),
                    translation.getY(),
                    angularVelocity);
        }

        SwerveModuleState[] swerveModuleStates = swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveContants.FALCON_MAX_SPEED_MPS);

        setModulesState(swerveModuleStates, true);
    }

    public void setModulesState(SwerveModuleState[] moduleStates, boolean preventJittering) {
        fieldsTable.recordOutput(
                "Module Desired States",
                moduleStates[0],
                moduleStates[1],
                moduleStates[2],
                moduleStates[3]);

        for (SwerveModule module : modules) {
            module.setDesiredState(moduleStates[module.getModuleNumber()], preventJittering);
        }
    }

    public double getYaw() {
        return odometry.getPoseMeters().getRotation().getDegrees();
    }

    public void requestResetModulesToAbsolute() {
        for (SwerveModule module : modules) {
            module.queueResetToAbsolute();
        }
    }

    public Rotation2d getRotation2d() {
        return odometry.getPoseMeters().getRotation();
    }

    public SwerveModulePosition[] getModulesPositions() {
        SwerveModulePosition[] modulePosition = new SwerveModulePosition[4];

        for (SwerveModule module : modules) {
            modulePosition[module.getModuleNumber()] = new SwerveModulePosition(
                    module.getDriveDistanceMeters(),
                    new Rotation2d(Math.toRadians(module.getAbsoluteAngleDegrees())));
        }

        return modulePosition;
    }
}
