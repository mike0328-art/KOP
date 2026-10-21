// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.shootConstand;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  private final SparkMax LFMotor;
  private final SparkMax RFMotor;
  private final SparkMax LBMotor;
  private final SparkMax RBMotor;

  private final TalonFX shootMotorL;
  private final TalonFX shootMotorR;

  private final TalonFXConfiguration shootMotorConfigL;
  private final TalonFXConfiguration shootMotorConfigR;
  
  private double LeftshootSpeed;
  private double RightshootSpeed;
  private double shootStop;
  private boolean TempsoHigh;

  private final SparkMaxConfig LFConfig;
  private final SparkMaxConfig RFConfig;
  private final SparkMaxConfig LBConfig;
  private final SparkMaxConfig RBConfig;

  private final DifferentialDrive drive;
  public DriveSubsystem() {
    LFMotor = new SparkMax(1, MotorType.kBrushless);
    LBMotor = new SparkMax(2, MotorType.kBrushless);
    RFMotor = new SparkMax(3, MotorType.kBrushless);
    RBMotor = new SparkMax(4, MotorType.kBrushless);
    
    shootMotorL = new TalonFX(shootConstand.shootMotorL);
    shootMotorR = new TalonFX(shootConstand.shootMotorR);

    shootMotorConfigL = new TalonFXConfiguration();
    shootMotorConfigR = new TalonFXConfiguration();

    shootMotorConfigL.CurrentLimits.SupplyCurrentLimit = shootConstand.MaxOutput;
    shootMotorConfigR.CurrentLimits.SupplyCurrentLimit = shootConstand.MaxOutput;
    shootMotorConfigL.CurrentLimits.SupplyCurrentLimitEnable = true;
    shootMotorConfigR.CurrentLimits.SupplyCurrentLimitEnable = true;
    shootMotorConfigL.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shootMotorConfigR.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shootMotorConfigL.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    shootMotorConfigR.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    shootMotorL.getConfigurator().apply(shootMotorConfigL);
    shootMotorR.getConfigurator().apply(shootMotorConfigR);

    LFConfig = new SparkMaxConfig();
    LBConfig = new SparkMaxConfig();
    RFConfig = new SparkMaxConfig();
    RBConfig = new SparkMaxConfig();

    drive =  new DifferentialDrive(LFMotor, RFMotor);

    LFConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(60);
    LBConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(60).follow(LFMotor.getDeviceId());
    RFConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(60);
    RBConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(60).follow(RFMotor.getDeviceId());

    
  }
  public void manualDrive(Double xspeed, double turnspeed) {
    drive.arcadeDrive(xspeed, turnspeed);}

  public void strongshoot() {
    LeftshootSpeed = 12;
    RightshootSpeed = 12;
  }
  public void weakshoot() {
    LeftshootSpeed = 6;
    RightshootSpeed = 6;
  }
  public void stopshoot() {
    LeftshootSpeed = 0;
    RightshootSpeed = 0;
  }
  public double getRightShootTemp() {
    return shootMotorR.getDeviceTemp().getValueAsDouble();
  }
  public double getLeftShootTemp() {
    return shootMotorL.getDeviceTemp().getValueAsDouble();
  }
  public double getLeftShootVelocity() {
    return shootMotorL.getVelocity().getValueAsDouble();
  }
  public double getRightShootVelocity() {
    return shootMotorR.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    TempsoHigh = getLeftShootTemp() > 60 || getRightShootTemp() > 60;
    if (TempsoHigh) shootMotorL.stopMotor();
    else shootMotorL.setVoltage(LeftshootSpeed);
    
    if (TempsoHigh) shootMotorR.stopMotor();
    else shootMotorR.setVoltage(RightshootSpeed);
      
  }
}
