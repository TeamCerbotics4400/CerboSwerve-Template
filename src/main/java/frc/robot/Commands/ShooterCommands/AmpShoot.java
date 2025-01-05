package frc.robot.Commands.ShooterCommands;

import static frc.robot.Constants.Shooter.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;

public class AmpShoot extends Command {

  ShooterSubsystem shooter;
  IntakeSubsystem intake;

  public AmpShoot(ShooterSubsystem shooter, IntakeSubsystem intake) {
    this.shooter = shooter;
    this.intake = intake;

    addRequirements(shooter, intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    shooter.velocity(UPPER_SHOOTER_AMP_RPM, LOWER_SHOOTER_AMP_RPM);
    intake.setIntake(1);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopMotors();
    intake.stopIntake();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
