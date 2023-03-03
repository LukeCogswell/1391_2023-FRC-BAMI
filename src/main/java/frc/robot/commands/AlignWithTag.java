// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.SwerveModuleConstants.PID.*;

public class AlignWithTag extends CommandBase {
  private Drivetrain m_drivetrain;
  private Integer node;
  private PIDController yController = new PIDController(0.04, kDriveI, kDriveD);
  private PIDController xController = new PIDController(0.25, kDriveI, kDriveD);
  private PIDController turnController = new PIDController(1.5, kSteerI, kSteerD);

  /**
   * Looks for the nearest node April tag and takes a given node number and drive toward it.
   * 
   * @param drivetrain - the current {@link Drivetrain}
   * @param whichNode - the desired node to travel to. 
   * Integer values of 1, 2 and 3 are applicable, with 1 and 2 being cone nodes.
   */
  public AlignWithTag(Drivetrain drivetrain, Integer whichNode) {
    node = whichNode;
    m_drivetrain = drivetrain;

    addRequirements(m_drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_drivetrain.botPoseSub.get().length != 6) {
      System.out.println("STOPPINGGGGGG");
      this.cancel();
      return;
    } 

    m_drivetrain.updateOdometryIfTag();
    m_drivetrain.limelightToTagMode();
    turnController.enableContinuousInput(-180, 180);
    turnController.setTolerance(0.5);
    xController.setTolerance(0.5);
    yController.setTolerance(0.5);

    if (node == 2) m_drivetrain.limelightToTagMode();
    else m_drivetrain.limelightToTapeMode();

    if (m_drivetrain.getTV() == 0) {
      this.cancel();
      return;
    }

    xController.setSetpoint(3.4);
    turnController.setSetpoint(0.0);
    yController.setSetpoint(1.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    var xDrive = -xController.calculate(m_drivetrain.getTA());
    var yDrive = -yController.calculate(m_drivetrain.getTX());
    var rot = turnController.calculate(m_drivetrain.getFieldPosition().getRotation().getRadians());
    rot = MathUtil.clamp(rot, -1, 1);
    xDrive = MathUtil.clamp(xDrive, -3.0, 3.0);
    yDrive = MathUtil.clamp(yDrive, -3.0, 3.0);
    if (m_drivetrain.getTA() < 0.5) xDrive = -2.5;
    if (m_drivetrain.getTA() == 0) xDrive = 0.0;
    m_drivetrain.drive(xDrive, yDrive, rot, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      m_drivetrain.limelightToTagMode();
    }
    turnController.close();
    yController.close();
    xController.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (xController.atSetpoint() && yController.atSetpoint() && turnController.atSetpoint()) {
      return true;
    }
    return false;
  }
}
