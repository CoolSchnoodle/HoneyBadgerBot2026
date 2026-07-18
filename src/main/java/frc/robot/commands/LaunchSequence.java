// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.FuelSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LaunchSequence extends SequentialCommandGroup {
  /** Creates a new LaunchSequence. */
  public LaunchSequence(FuelSubsystem fuelSubsystem) {
    Command spinUp = new SpinUp(fuelSubsystem,
      SmartDashboard.getNumber("Spin-up launcher rps", Constants.FuelConstants.SPIN_UP_LAUNCHER_ROTATIONS_PER_SECOND))
      .until(() -> fuelSubsystem.launcherPIDReady());
    Launch launch = new Launch(fuelSubsystem,
      SmartDashboard.getNumber("Launching launcher rps", Constants.FuelConstants.LAUNCHING_LAUNCHER_ROTATIONS_PER_SECOND));
    addCommands(spinUp, launch);
  }
  public LaunchSequence(FuelSubsystem fuelSubsystem, double launchingRps) {
    Command spinUp = new SpinUp(fuelSubsystem, launchingRps + 0)
      .until(() -> fuelSubsystem.launcherPIDReady());
    Launch launch = new Launch(fuelSubsystem, launchingRps); 
    addCommands(spinUp, launch);
  }
  public LaunchSequence(FuelSubsystem fuelSubsystem, double launchingRps, double spinUpRps) {
    Command spinUp = new SpinUp(fuelSubsystem, spinUpRps)
      .until(() -> fuelSubsystem.launcherPIDReady());
    Launch launch = new Launch(fuelSubsystem, launchingRps); 
    addCommands(spinUp, launch);
  }
}
