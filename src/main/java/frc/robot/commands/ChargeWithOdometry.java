// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.SwerveModuleConstants.PID.*;

public class ChargeWithOdometry extends CommandBase {
  private Drivetrain m_drivetrain;
  private PIDController xController = new PIDController(kDriveP, kDriveI, kDriveD);
  private Double setpoint;
  /** Creates a new ChargeWithOdometry. */
  public ChargeWithOdometry(Drivetrain drivetrain, Double x) {
    m_drivetrain = drivetrain;
    setpoint = x;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController.setSetpoint(setpoint);
    xController.setTolerance(0.2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var xDrive = xController.calculate(m_drivetrain.getFieldPosition().getX());
    xDrive = MathUtil.clamp(xDrive, -1, 1);
    m_drivetrain.drive(xDrive, 0, 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    xController.close();
    m_drivetrain.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (xController.atSetpoint()) return true;
    else return false;
  }
}
