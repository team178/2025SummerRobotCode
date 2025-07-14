// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.SwerveDrive;

public class RobotContainer {
    private CommandXboxController driverController = new CommandXboxController(0);
    // private CommandXboxController auxController = new CommandXboxController(0);

    // subsystems
    private SwerveDrive swerve;
    
    public RobotContainer() {
        swerve = new SwerveDrive();

        configureBindings();
    }

    private void configureBindings() {
        swerve.setDefaultCommand(swerve.runControllerInputs(
            driverController::getLeftX,
            driverController::getLeftY,
            driverController::getRightX
        ));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
