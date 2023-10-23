package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;

public class SwerveContants {

    public final static double GEAR_RATIO_DRIVE = 6.75;
    public final static double GEAR_RATIO_ANGLE = 12.6;
    public final static double WHEEL_CIRCUMFERENCE = 0.005;

    // public final static double FALCON_MAX_SPEED = Converstions.falconToMPS(Converstions.RPMToFalcon(6040, GEAR_RATIO_DRIVE), WHEEL_CIRCUMFERENCE, GEAR_RATIO_DRIVE);
    public final static double FALCON_MAX_SPEED = 1;
    public final static double FALCOM_MAX_ANGULAR_VELOCITY = 0.1;

    public final static Translation2d FRONT_RIGHT_LOCATION = new Translation2d(0.551942 / 2, 0.551942 / 2);
    public final static Translation2d FRONT_LEFT_LOCATION = new Translation2d(0.551942 / 2, -0.551942 / 2);
    public final static Translation2d BACK_RIGHT_LOCATION = new Translation2d(-0.551942 / 2, 0.551942 / 2);
    public final static Translation2d BACK_LEFT_LOCATION = new Translation2d(-0.551942 / 2, -0.551942 / 2);

    public final static double MODULE_0_ANGLE_OFFSET_DEGREES = 0;
    public final static double MODULE_1_ANGLE_OFFSET_DEGREES = 0;
    public final static double MODULE_2_ANGLE_OFFSET_DEGREES = 0;
    public final static double MODULE_3_ANGLE_OFFSET_DEGREES = 0;

}
