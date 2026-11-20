// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.driveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.driveSubsystem;

public class Autos extends SubsystemBase{
    private static final double kDriveSpeed = 0.5;
    private static final double kTurnSpeed = 0.5;

    private final driveSubsystem driveSubsystem;

    public Autos(driveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
    }

    // These set the drivetrain output once. Call them repeatedly (e.g. from a
    // command's execute) and call stop() when finished.

    public void driveForwards() {
        driveSubsystem.Drive(kDriveSpeed, 0);
    }

    public void driveBackwards() {
        driveSubsystem.Drive(-kDriveSpeed, 0);
    }

    /** Spins in place, counterclockwise. */
    public void turnLeft() {
        driveSubsystem.Drive(0, kTurnSpeed);
    }

    /** Spins in place, clockwise. */
    public void turnRight() {
        driveSubsystem.Drive(0, -kTurnSpeed);
    }

    public void stop() {
        driveSubsystem.Drive(0, 0);
    }
}