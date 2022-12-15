package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class HoodShot extends CommandBase {
  /** Creates a new HoodShot. */
  ShooterSubsystem shooterSubsystem;
  double ty, theta, dist, vel, hAng;
  double hoodAngleSetPoint = 10;
  public HoodShot(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    //shooterSubsystem.hoodAngle(hAng);
  }

  @Override
  public void execute() {
    //ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    //theta = Constants.limeangle+ty;
    //dist = Constants.goalht/Math.tan(Math.toRadians(theta));
    //vel = 1000; //add regression w/ dist as input
    //hAng = ty+19.0;
    //shooterSubsystem.hoodAngle(hAng);
    dist = shooterSubsystem.dist;
    vel = shooterSubsystem.getPowDemand(dist);
    SmartDashboard.putNumber("SRPM", vel);

    shooterSubsystem.shootRPM(vel);
    

  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.shootRPM(0);
    // hoodAngleSetPoint = hoodAngleSetPoint + 5;
    //shooterSubsystem.hoodAngle(0);
    shooterSubsystem.loadBall.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}