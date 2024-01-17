package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.tuneables.TuneableCommand;
import frc.lib.tuneables.TuneablesManager;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveCommands;
import frc.robot.utils.NaturalXboxController;

public class RobotContainer {
    private final Swerve swerve = new Swerve();
    private final NaturalXboxController driverController = new NaturalXboxController(RobotMap.Controllers.DRIVER_PORT);
    private final SwerveCommands swerveCommands = new SwerveCommands(swerve);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        TuneableCommand driveCommand = swerveCommands.controller(
                driverController::getSquaredLeftY,
                driverController::getSquaredLeftX,
                driverController::getSquaredRightX,
                driverController.leftBumper().negate()::getAsBoolean);

        swerve.setDefaultCommand(driveCommand);
        TuneablesManager.add("Swerve/drive command", driveCommand.fullTuneable());

        driverController.a().onTrue(new InstantCommand(swerve::resetYaw));

        TuneablesManager.add("Swerve/modules control mode",
                swerveCommands.controlModules(
                        driverController::getLeftX,
                        driverController::getLeftY,
                        driverController::getRightX).fullTuneable());
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
