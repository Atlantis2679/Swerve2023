package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.io.GyroIO;
import frc.robot.subsystems.swerve.io.GyroIONavX;
import frc.robot.subsystems.swerve.io.GyroIOSim;
import frc.lib.logfields.LogFieldsTable;
import frc.lib.tuneables.SendableType;
import frc.lib.tuneables.Tuneable;
import frc.lib.tuneables.TuneableBuilder;
import frc.lib.tuneables.TuneablesManager;
import frc.lib.valueholders.DoubleHolder;
import frc.robot.Robot;
import frc.robot.RobotMap.Module0;
import frc.robot.RobotMap.Module1;
import frc.robot.RobotMap.Module2;
import frc.robot.RobotMap.Module3;

import static frc.robot.subsystems.swerve.SwerveContants.*;

public class Swerve extends SubsystemBase implements Tuneable {
    private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());

    private final GyroIO gyroIO = Robot.isSimulation()
            ? new GyroIOSim(fieldsTable.getSubTable("Gyro"))
            : new GyroIONavX(fieldsTable.getSubTable("Gyro"));

    private final SwerveDriveOdometry odometry;

    // Should be FL, FR, BL, BR
    private final SwerveModule[] modules = {
            new SwerveModule(0, Module3.DRIVE_MOTOR_ID, Module3.ANGLE_MOTOR_ID, Module3.ENCODER_ID,
                    MODULE_3_ABSOLUTE_ANGLE_OFFSET_DEGREES, fieldsTable),
            new SwerveModule(1, Module0.DRIVE_MOTOR_ID, Module0.ANGLE_MOTOR_ID, Module0.ENCODER_ID,
                    MODULE_0_ABSOLUTE_ANGLE_OFFSET_DEGREES, fieldsTable),
            new SwerveModule(2, Module2.DRIVE_MOTOR_ID, Module2.ANGLE_MOTOR_ID, Module2.ENCODER_ID,
                    MODULE_2_ABSOLUTE_ANGLE_OFFSET_DEGREES, fieldsTable),
            new SwerveModule(3, Module1.DRIVE_MOTOR_ID, Module1.ANGLE_MOTOR_ID, Module1.ENCODER_ID,
                    MODULE_1_ABSOLUTE_ANGLE_OFFSET_DEGREES, fieldsTable)
    };

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
            -SwerveContants.TRACK_LENGTH_M / 2,
            SwerveContants.TRACK_WIDTH_M / 2);
    public final Translation2d BACK_RIGHT_LOCATION = new Translation2d(
            -SwerveContants.TRACK_WIDTH_M / 2,
            -SwerveContants.TRACK_LENGTH_M / 2);

    private final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            FRONT_LEFT_LOCATION,
            FRONT_RIGHT_LOCATION,
            BACK_LEFT_LOCATION,
            BACK_RIGHT_LOCATION);

    private double currYawDegreesCW = 0;
    private double yawOffsetDegreesCW = 0;

    public Swerve() {
        fieldsTable.update();

        odometry = new SwerveDriveOdometry(
                swerveKinematics,
                gyroIO.isConnected.getAsBoolean() ? new Rotation2d(gyroIO.yawDegreesCW.getAsDouble())
                        : new Rotation2d(0),
                getModulesPositions());

        TuneablesManager.add("Swerve", (Tuneable) this);
    }

    @Override
    public void periodic() {
        for (SwerveModule module : modules) {
            module.periodic();
        }

        if (gyroIO.isConnected.getAsBoolean()) {
            currYawDegreesCW = gyroIO.yawDegreesCW.getAsDouble();
        } else {
            Twist2d twist = swerveKinematics.toTwist2d(
                    modules[0].getModulePositionDelta(),
                    modules[1].getModulePositionDelta(),
                    modules[2].getModulePositionDelta(),
                    modules[3].getModulePositionDelta());

            currYawDegreesCW += Math.toDegrees(-twist.dtheta);
        }

        odometry.update(Rotation2d.fromRadians(Math.toRadians(-getCurrYawDegreesCW())), getModulesPositions());

        fieldsTable.recordOutput("Odometry", odometry.getPoseMeters());
        fieldsTable.recordOutput("Module States",
                modules[0].getModuleState(),
                modules[1].getModuleState(),
                modules[2].getModuleState(),
                modules[3].getModuleState());

        fieldsTable.recordOutput("Module States Integreated",
                modules[0].getModuleStateIntegreated(),
                modules[1].getModuleStateIntegreated(),
                modules[2].getModuleStateIntegreated(),
                modules[3].getModuleStateIntegreated());

        fieldsTable.recordOutput("Robot Yaw Radians CWW", -Math.toRadians(getCurrYawDegreesCW()));
    }

    public void drive(double forward, double sidewaysRightPositive, double angularVelocityCW, boolean isFieldRelative) {
        ChassisSpeeds desiredChassisSpeeds;

        double angularVelocityCCW = -angularVelocityCW;
        double sidewaysLeftPositive = -sidewaysRightPositive;

        if (isFieldRelative) {
            desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    forward,
                    sidewaysLeftPositive,
                    angularVelocityCCW,
                    Rotation2d.fromDegrees(-getCurrYawDegreesCW()));
        } else {
            desiredChassisSpeeds = new ChassisSpeeds(
                    forward,
                    sidewaysLeftPositive,
                    angularVelocityCCW);
        }

        SwerveModuleState[] swerveModuleStates = swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveContants.MAX_SPEED_MPS);

        setModulesState(swerveModuleStates, true, true);
    }

    public void setModulesState(SwerveModuleState[] moduleStates, boolean preventJittering, boolean optimizeState) {
        fieldsTable.recordOutput(
                "Module Desired States",
                moduleStates[0],
                moduleStates[1],
                moduleStates[2],
                moduleStates[3]);

        for (SwerveModule module : modules) {
            module.setDesiredState(moduleStates[module.getModuleNumber()], preventJittering, optimizeState);
        }
    }

    private double getCurrYawDegreesCW() {
        return currYawDegreesCW - yawOffsetDegreesCW;
    }

    public void setCurrYawDegreesCW(double newYawDegreesCW) {
        Pose2d currentPose = odometry.getPoseMeters();
        yawOffsetDegreesCW = currYawDegreesCW - newYawDegreesCW;

        odometry.resetPosition(
                Rotation2d.fromDegrees(-getCurrYawDegreesCW()),
                getModulesPositions(),
                new Pose2d(
                        currentPose.getX(),
                        currentPose.getY(),
                        new Rotation2d(-newYawDegreesCW)));
    }

    public void resetYaw() {
        setCurrYawDegreesCW(0);
    }

    public void requestResetModulesToAbsolute() {
        for (SwerveModule module : modules) {
            module.queueResetToAbsolute();
        }
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

    @Override
    public void initTuneable(TuneableBuilder builder) {
        builder.addChild("Swerve Subsystem", (Sendable) this);

        builder.addChild("Modules Angle PID", (Tuneable) (builderPID) -> {
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
        builder.addChild("Module 0", modules[0]);
        builder.addChild("Module 1", modules[1]);
        builder.addChild("Module 2", modules[2]);
        builder.addChild("Module 3", modules[3]);

        builder.addChild("reset modules absolute position", (Tuneable) (resetModulesBuilder) -> {
            DoubleHolder angleToResetDegrees = new DoubleHolder(0);
            resetModulesBuilder.addDoubleProperty("angle to reset degrees", angleToResetDegrees::get,
                    angleToResetDegrees::set);
            resetModulesBuilder.addChild("reset!", new InstantCommand(() -> {
                for (SwerveModule swerveModule : modules) {
                    swerveModule.setAbsoluteEncoderAngleDegrees(angleToResetDegrees.get());
                }
            }));
        });

        builder.addChild("coast mode", new RunCommand(() -> {
            for (SwerveModule module : modules) {
                module.enableCoastMode();
            }
        }).ignoringDisable(true));

        builder.addChild("reset to absolute", new InstantCommand(this::requestResetModulesToAbsolute));
    }
}
