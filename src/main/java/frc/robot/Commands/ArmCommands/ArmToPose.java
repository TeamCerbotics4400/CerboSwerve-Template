// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Arm.ArmSubsystem;

public class ArmToPose extends Command {
  /** Creates a new ArmToPose. */
  ArmSubsystem m_arm;

  double angle = 0.0;

  public ArmToPose(ArmSubsystem m_arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_arm = m_arm;

    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.getController().reset(m_arm.getArmAngle());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_arm.updateArmSetpoint(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.updateArmSetpoint(160.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
