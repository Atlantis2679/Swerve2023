// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.tuneables.SendableType;
import frc.lib.tuneables.TuneableCommand;
import frc.lib.tuneables.TuneablesManager;
import frc.lib.tuneables.TuneablesTable;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.commands.SwerveCommands;

public class RobotContainer {
    private final TuneablesTable swerveTunenablesCommandTable = new TuneablesTable(SendableType.LIST);
    private final Swerve swerve = new Swerve();
    private final CommandXboxController driverController = new CommandXboxController(RobotMap.Controllers.DRIVER_PORT);
    private final SwerveCommands swerveCommands = new SwerveCommands(swerve);

    public RobotContainer() {
        configureBindings();
        TuneablesManager.add("SwerveCommands", swerveTunenablesCommandTable);
    }

    private void configureBindings() {
        swerve.setDefaultCommand(registerSwerveCommand("controller", swerveCommands.controller(
                () -> MathUtil.applyDeadband(driverController.getLeftX(), 0.05),
                () -> MathUtil.applyDeadband(-driverController.getLeftY(), 0.05),
                () -> MathUtil.applyDeadband(driverController.getRightX(), 0.05),
                () -> driverController.leftBumper().getAsBoolean())));

        registerSwerveCommand("control modules",
                swerveCommands.controlModules(
                        () -> MathUtil.applyDeadband(driverController.getLeftX(), 0.1),
                        () -> MathUtil.applyDeadband(-driverController.getLeftY(), 0.1)));
    }

    private TuneableCommand registerSwerveCommand(String name, TuneableCommand command) {
        swerveTunenablesCommandTable.addChild(name, command.fullTuneable());
        return command;
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
