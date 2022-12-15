package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
  IntakeSubsystem intakeSubsystem;

  public IntakeCommand(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsystem.compressor.enableDigital();
  }

  @Override
  public void execute() {

      if (intakeSubsystem.ballColor == Constants.correctBall && intakeSubsystem.detectBeamBreaker2() == true && intakeSubsystem.detectBeamBreaker3()==true)/* 2 correct balls*/{
        //STOP intake
        intakeSubsystem.stopIntake();
      }
      else {
        //Intake Starts
        intakeSubsystem.intakeBall(0.3);
        intakeSubsystem.intakeSolenoid.set(true);
        SmartDashboard.putNumber("Intake RPM", (60*IntakeSubsystem.ballIntake.getSelectedSensorVelocity()/(Constants.sensorPosititons)));
        
        // intakeSubsystem.feedBall();

        // Feeder Starts
         if (intakeSubsystem.detectBeamBreaker1() == true || intakeSubsystem.detectBeamBreaker2()==true) {
         intakeSubsystem.feedBall();

        //Detecting Color
         if ((int)intakeSubsystem.detectBallColor() == Constants.correctBall)  {
          //Get ball UP
          intakeSubsystem.uptakeBall();
        }
         else if ((int)intakeSubsystem.detectBallColor() != Constants.correctBall)  {
          //Get ball OUT
          intakeSubsystem.outtakeBall();
        }
      }
  }
    SmartDashboard.putNumber("ball color", intakeSubsystem.detectBallColor());
  }

  @Override
  public void end(boolean interrupted) {

    intakeSubsystem.stopIntake();
    // close everything
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
