package frc.robot.Commands.ShooterCommands;

import static frc.robot.Constants.Shooter.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;

public class SpeakerShoot extends Command {

  ShooterSubsystem shooter;

  public SpeakerShoot(ShooterSubsystem shooter) {
    this.shooter = shooter;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    shooter.velocity(UPPER_SHOOTER_SPEAKER_RPM, LOWER_SHOOTER_SPEAKER_RPM);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
