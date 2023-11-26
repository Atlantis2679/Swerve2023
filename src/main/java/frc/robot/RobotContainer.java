// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.commands.TeleopSwerve;

public class RobotContainer {
    private final static Swerve swerve = new Swerve();
    public final XboxController driverController = new XboxController(RobotMap.Controllers.DRIVER_PORT);
    public final JoystickButton robotCentric = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        swerve.setDefaultCommand(new TeleopSwerve(
                swerve,
                () -> driverController.getLeftX(),
                () -> driverController.getLeftY(),
                () -> driverController.getRightX(),
                () -> robotCentric.getAsBoolean()));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
