package frc.robot.subsystems.swerve;

public class Converstions {
    private final static double COUNTS_PER_REV_CANCODER = 4096;
    private final static double COUNTS_PER_REV_FALCON = 2048;

    public static double radiusToCircumstance(double radius) {
        return radius * 2 * Math.PI;
    }

    public static double CANcoderToDegrees(double positionCounts, double gearRatio) {
        return positionCounts * (360.0 / gearRatio);
    }

    public static double degreesToCANcoder(double degrees, double gearRatio) {
        return degrees / (360.0 / gearRatio);
    }

    public static double falconToDegrees(double positionCounts, double gearRatio) {
        return positionCounts * (360.0 / gearRatio);
    }

    public static double degreesToFalcon(double degrees, double gearRatio) {
        return degrees / (360.0 / gearRatio);
    }

    public static double RPMToDegrees(double degrees, double gearRatio) {
        return falconToDegrees(RPMToFalcon(degrees, gearRatio), gearRatio);
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

    public static double falconToMPS(double velocitycounts, double wheelRadiusMeters, double gearRatio) {
        double wheelRPM = falconToRPM(velocitycounts, gearRatio);
        double wheelMPS = (wheelRPM * radiusToCircumstance(wheelRadiusMeters)) / 60;
        return wheelMPS;
    }

    public static double MPSToFalcon(double velocity, double wheelRadiusMeters, double gearRatio) {
        double wheelRPM = ((velocity * 60) / radiusToCircumstance(wheelRadiusMeters));
        double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
        return wheelVelocity;
    }

    public static double falconToMeters(double positionCounts, double wheelRadiusMeters, double gearRatio) {
        return positionCounts * (radiusToCircumstance(wheelRadiusMeters) / (gearRatio * COUNTS_PER_REV_FALCON));
    }

    public static double MetersToFalcon(double meters, double wheelRadiusMeters, double gearRatio) {
        return meters / (radiusToCircumstance(wheelRadiusMeters) / (gearRatio * COUNTS_PER_REV_FALCON));
    }

    public static double RPMToMPS(double RPM, double wheelRadiusMeters) {
        return wheelRadiusMeters * 2 * Math.PI * (RPM / 60);
    }
}
