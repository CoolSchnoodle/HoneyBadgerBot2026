package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FuelSubsystem;

public class DepotAuto extends SequentialCommandGroup {
    DriveSubsystem driveSubsystem;
    FuelSubsystem fuelSubsystem;
    public DepotAuto(DriveSubsystem drive, FuelSubsystem fuel) {
        this.driveSubsystem = drive;
        this.fuelSubsystem = fuel;
        addCommands(
            new ParallelCommandGroup(
                new AutoDrive(driveSubsystem, 0.2, 0),
                new Intake(fuelSubsystem)
            ).withDeadline(new WaitCommand(7)),
            new AutoDrive(driveSubsystem, 0,-1)
                .withDeadline(new WaitCommand(1)),
            driveSubsystem.rotateToHubCommand()
                .withDeadline(new WaitCommand(3)),
            fuelSubsystem.shootAuto(drive, 5)
        );
    }
}
