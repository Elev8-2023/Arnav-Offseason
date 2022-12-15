package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.HoodShot;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.TurretCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {

  //Subsystems
  DriveSubsystem driveSubsystem = new DriveSubsystem();
  IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  //Commands
  DriveCommand driveCommand = new DriveCommand(driveSubsystem);
  IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem);
  HoodShot hoodShot = new HoodShot(shooterSubsystem);

  //IO
  public static XboxController driveController = new XboxController(Constants.driveControllerPort);
  public static JoystickButton intakeButton = new JoystickButton(driveController, 5);
  public static JoystickButton shooterButton = new JoystickButton(driveController, 6);

  public RobotContainer() {

    configureButtonBindings();
    driveSubsystem.setDefaultCommand(driveCommand);
  }

  private void configureButtonBindings() {
    intakeButton.toggleWhenActive(new IntakeCommand(intakeSubsystem));
    shooterButton.toggleWhenActive(new HoodShot(shooterSubsystem));

  }

  public Command getAutonomousCommand() {
    return null;
  }

  public static double getxSpeed(){
    double xSpeed = driveController.getRawAxis(1);
    return xSpeed;
  }

  public static double getzRotation(){
    double zRotation = driveController.getRawAxis(4);
    return zRotation*-1;
  }

}
