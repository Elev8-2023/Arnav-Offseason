package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {

  DriveSubsystem driveSubsystem;

  public DriveCommand(DriveSubsystem driveSubsystem) {

    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);

  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    driveSubsystem.drive(RobotContainer.getxSpeed(), RobotContainer.getzRotation());
    
  }

  @Override
  public void end(boolean interrupted) {
    
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
