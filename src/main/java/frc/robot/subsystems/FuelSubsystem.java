// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.FuelConstants.*;

public class FuelSubsystem extends SubsystemBase {
  private final TalonFX feederRoller;
  private final TalonFX intakeLauncherRoller;

  /** Creates a new CANBallSubsystem. */
  public FuelSubsystem() {
    // create brushed motors for each of the motors on the launcher mechanism
    intakeLauncherRoller = new TalonFX(INTAKE_LAUNCHER_MOTOR_ID);
    feederRoller = new TalonFX(FEEDER_MOTOR_ID);

    // create the configuration for the feeder roller, set a current limit and apply
    // the config to the controller
    TalonFXConfiguration feederConfig = new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
      .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT));
    feederRoller.getConfigurator().apply(feederConfig);

    // create the configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for both intaking and
    // launching, and apply the config to the controller

    TalonFXConfiguration launcherConfig = new TalonFXConfiguration()
        .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
        .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT));
    intakeLauncherRoller.getConfigurator().apply(launcherConfig);

    // put default values for various fuel operations onto the dashboard
    // all commands using this subsystem pull values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
    //SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE);
    SmartDashboard.putNumber("Launching launcher rotations per second", LAUNCHING_LAUNCHER_ROTATIONS_PER_SECOND);
    SmartDashboard.putNumber("Spin-Up launcher velocity setpoint", SPIN_UP_LAUNCHER_ROTATIONS_PER_SECOND);
    SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);
  }

  // A method to set the voltage of the intake roller
  public void setIntakeLauncherRoller(double voltage) {
    intakeLauncherRoller.setControl(new VoltageOut(voltage));
  }

  // A method to set the voltage of the intake roller
  public void setFeederRoller(double voltage) {
    feederRoller.setControl(new VoltageOut(voltage));
  }

  // A method to stop the rollers
  public void stop() {
    feederRoller.set(0);
    intakeLauncherRoller.set(0);
  }

  @Override
  public void periodic() {
  }

  public void setLauncherPID(double velocitySetpoint) {
    intakeLauncherRoller.setControl(new VelocityDutyCycle(RotationsPerSecond.of(velocitySetpoint)));    
  }
  
  public boolean launcherPIDReady() {
    return intakeLauncherRoller.getClosedLoopError().getValueAsDouble() < 2;
  }

  public double luancherVelocity() {
    return intakeLauncherRoller.getVelocity().getValueAsDouble();
  }

  public void setIntakeLauncherSpeed(double speed) {
    intakeLauncherRoller.set(speed);
  }
}
