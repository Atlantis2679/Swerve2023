package frc.robot;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;
    private static List<Runnable> periodics = new ArrayList<>();

    public static void registerPeriodic(Runnable runnable) {
        periodics.add(runnable);
    }

    private String getLogPath() {
        if (isReal()) {
            try {
                // prefer a mounted USB drive if one is accessible
                Path usbDir = Paths.get("/u").toRealPath();
                if (Files.isWritable(usbDir)) {
                    return usbDir.toString();
                }
            } catch (IOException ex) {
                // ignored
            }
        }
        String path = Filesystem.getOperatingDirectory().getAbsolutePath();
        return isReal() ? path : path + "\\logs";
    }

    private void initializeAdvantageKit() {
        Logger logger = Logger.getInstance();

        logger.recordMetadata("RuntimeType", getRuntimeType().toString());
        logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

        if (isSimulation() && Constants.REPLAY) {
            setUseTiming(false);
            String logPath = LogFileUtil.findReplayLog();
            logger.setReplaySource(new WPILOGReader(logPath));
            logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay")));

        } else {
            String logPath = getLogPath();

            logger.addDataReceiver(new WPILOGWriter(logPath));
            logger.addDataReceiver(new NT4Publisher() {
                @Override
                public void putTable(LogTable table) {
                    if (table.getBoolean("DriverStation/Test", false))
                        super.putTable(table);
                }
            });
            LoggedPowerDistribution.getInstance(0, ModuleType.kCTRE);
        }

        logger.start();
    }

    @Override
    public void robotInit() {
        initializeAdvantageKit();
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        for (Runnable periodic : periodics) {
            periodic.run();
        }
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().enable();
        teleopInit();
    }

    @Override
    public void testPeriodic() {
        teleopPeriodic();
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}