// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualDrive extends Command {
  /** Creates a new ManualDrive. */
  private final DriveSubsystem m_DriveSubsystem;
  private final DoubleSupplier xspeedfunc;
  private final DoubleSupplier turnspeedfunc;

  private double xspeed;
  private double turnspeed;

  public ManualDrive(DriveSubsystem driveSubsystem, DoubleSupplier xspeedfunc, DoubleSupplier turnspeedfunc) {
    this.m_DriveSubsystem = driveSubsystem;
    this.xspeedfunc = xspeedfunc;
    this.turnspeedfunc = turnspeedfunc;
    addRequirements(m_DriveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xspeed = xspeedfunc.getAsDouble();
    turnspeed = turnspeedfunc.getAsDouble();
    m_DriveSubsystem.manualDrive(xspeed*0.6, turnspeed*0.6);
    m_DriveSubsystem.strongshoot();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveSubsystem.manualDrive(0.0, 0.0);
    m_DriveSubsystem.stopshoot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
