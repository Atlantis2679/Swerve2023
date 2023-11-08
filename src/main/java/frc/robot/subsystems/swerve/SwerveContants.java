package frc.robot.subsystems.swerve;

import edu.wpi.first.math.util.Units;

public class SwerveContants {

    public final static double GEAR_RATIO_DRIVE = 6.75;
    public final static double GEAR_RATIO_ANGLE = 12.6;
    public final static double WHEEL_RADIUS_M = Units.inchesToMeters(2);
    public final static double WHEEL_CIRCUMFERENCE_M = WHEEL_RADIUS_M * 2 * Math.PI;


    public final static double FALCON_MAX_SPEED_MPS = 2;
    public final static double FALCOM_MAX_ANGULAR_VELOCITY = 0.1;

    public final static double TRACK_LENGTH_M = 0.551942;
    public final static double TRACK_WIDTH_M = 0.551942;

    public final static double MODULE_0_ANGLE_OFFSET_DEGREES = 0;
    public final static double MODULE_1_ANGLE_OFFSET_DEGREES = 0;
    public final static double MODULE_2_ANGLE_OFFSET_DEGREES = 0;
    public final static double MODULE_3_ANGLE_OFFSET_DEGREES = 0;
}
