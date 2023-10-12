package frc.robot.subsystems.swerve;

public class Converstions {
    private final static double COUNTS_PER_REV_CANCODER = 4096;
    private final static double COUNTS_PER_REV_FALCON = 2048;

    public static double CANcoderToDegrees(double positionCounts, double gearRatio) {
        return positionCounts * (360.0 / (gearRatio * COUNTS_PER_REV_CANCODER));
    }

    public static double degreesToCANcoder(double degrees, double gearRatio) {
        return degrees / (360.0 / (gearRatio * COUNTS_PER_REV_CANCODER));
    }

    public static double falconToDegrees(double positionCounts, double gearRatio) {
        return positionCounts * (360.0 / (gearRatio * COUNTS_PER_REV_FALCON));
    }

    public static double degreesToFalcon(double degrees, double gearRatio) {
        return degrees / (360.0 / (gearRatio * COUNTS_PER_REV_FALCON));
    }

    public static double falconToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (600.0 / COUNTS_PER_REV_CANCODER);
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    public static double RPMToFalcon(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        double sensorCounts = motorRPM * (COUNTS_PER_REV_FALCON / 600.0);
        return sensorCounts;
    }

    public static double falconToMPS(double velocitycounts, double wheel_circumference, double gearRatio) {
        double wheelRPM = falconToRPM(velocitycounts, gearRatio);
        double wheelMPS = (wheelRPM * wheel_circumference) / 60;
        return wheelMPS;
    }

    public static double MPSToFalcon(double velocity, double wheel_circumference, double gearRatio) {
        double wheelRPM = ((velocity * 60) / wheel_circumference);
        double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
        return wheelVelocity;
    }

    public static double falconToMeters(double positionCounts, double wheel_circumference, double gearRatio) {
        return positionCounts * (wheel_circumference / (gearRatio * COUNTS_PER_REV_FALCON));
    }

    public static double MetersToFalcon(double meters, double wheel_circumference, double gearRatio) {
        return meters / (wheel_circumference / (gearRatio * COUNTS_PER_REV_FALCON));
    }

}
