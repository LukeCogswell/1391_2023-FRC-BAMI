// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.MeasurementConstants.*;
import static frc.robot.Constants.SwerveModuleConstants.PID.*;

public class AlignWithNode extends CommandBase {
  private Drivetrain m_drivetrain;
  private Integer node;
  private double targetYPos;
  private PIDController yController = new PIDController(kDriveP, kDriveI, kDriveD);
  private PIDController xController = new PIDController(kDriveP, kDriveI, kDriveD);
  private PIDController turnController = new PIDController(0.05, kSteerI, kSteerD);

  /**
   * Looks for the nearest node April tag and takes a given node number and drive toward it.
   * 
   * @param drivetrain - the current {@link Drivetrain}
   * @param whichNode - the desired node to travel to. 
   * Integer values of 1, 2 and 3 are applicable, with 1 and 3 being cone nodes.
   */
  public AlignWithNode(Drivetrain drivetrain, Integer whichNode) {
    node = whichNode;
    m_drivetrain = drivetrain;

    addRequirements(m_drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_drivetrain.getTV() == 0) {
      this.cancel();
      return;
    } 
    
    turnController.enableContinuousInput(-180, 180);
    turnController.setTolerance(2);
    xController.setTolerance(0.1);
    yController.setTolerance(0.1);

    m_drivetrain.updateOdometryIfTag();
    var tagID = m_drivetrain.getTID();
    var tagPose = Constants.AprilTagFieldLayouts.AprilTagList.get(tagID - 1).pose;
  
    // if (node == 2) m_drivetrain.limelightToTagMode();
    // else m_drivetrain.limelightToTapeMode();

    var offsetY = 0.0;
    xController.setSetpoint(1.8);
    turnController.setSetpoint(0);
    if (node == 3 ) offsetY = -kNodeOffset; 
    if (node == 1) offsetY = kNodeOffset;
    targetYPos = tagPose.getY() + offsetY;
  
    yController.setSetpoint(targetYPos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    var xDrive = xController.calculate(m_drivetrain.getFieldPosition().getX());
    var yDrive = yController.calculate(m_drivetrain.getFieldPosition().getY());
    var rot = turnController.calculate(m_drivetrain.getFieldPosition().getRotation().getDegrees());
    rot = MathUtil.clamp(rot, -0.7, 0.7);
    xDrive = MathUtil.clamp(xDrive, -.4, .4);
    yDrive = MathUtil.clamp(yDrive, -.4, .4);

    m_drivetrain.drive(xDrive, yDrive, rot, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0.0, 0.0, 0.0, true);
    // if (interrupted) {
    //   m_drivetrain.limelightToTagMode();
    // }
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
