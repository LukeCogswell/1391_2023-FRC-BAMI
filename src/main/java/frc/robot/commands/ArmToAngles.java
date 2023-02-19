// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import static frc.robot.Constants.ArmConstants.PID.*;
import static frc.robot.Constants.ArmConstants.*;

public class ArmToAngles extends CommandBase {
  private Arm m_arm;
  private Double targetShoulderAngle, targetElbowAngle;

  private PIDController shoulderController = new PIDController(kShoulderP, kShoulderI, kShoulderD);
  private PIDController elbowController = new PIDController(kElbowP, kElbowI, kElbowD);

  /** Creates a new ArmToAngle. */
  public ArmToAngles(Arm arm, Double shoulderAngle, Double elbowAngle) {
    m_arm = arm;
    targetElbowAngle = elbowAngle > 155 || elbowAngle < -155 ? 0.0: elbowAngle;
    targetShoulderAngle = shoulderAngle > 45 || shoulderAngle < -45 ? 0.0: shoulderAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shoulderController.reset();
    elbowController.reset();
    elbowController.setSetpoint(targetElbowAngle);
    shoulderController.setSetpoint(targetShoulderAngle);
    elbowController.setTolerance(0.5);
    shoulderController.setTolerance(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var elbowSpeed = -elbowController.calculate(m_arm.getElbowAngle());
    var shoulderSpeed = shoulderController.calculate(m_arm.getShoulderAngle());
    elbowSpeed = MathUtil.clamp(elbowSpeed, -kElbowMaxSpeed, kElbowMaxSpeed);
    shoulderSpeed = MathUtil.clamp(shoulderSpeed, -kShoulderMaxSpeed, kShoulderMaxSpeed);
    m_arm.setElbowMotors(elbowSpeed);
    m_arm.setShoulderMotors(-shoulderSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setElbowMotors(0.0);
    m_arm.setShoulderMotors(0.0);
    shoulderController.close();
    elbowController.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (shoulderController.atSetpoint() && elbowController.atSetpoint()) {
    //   return true;
    // }
    return false;
  }
}
