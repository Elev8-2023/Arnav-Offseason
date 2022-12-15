package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  public static TalonFX ballIntake = new TalonFX(2); 
  public static TalonFX ballIntake2 = new TalonFX(10); //omnis
  public static TalonFX ballFeeder = new TalonFX(8);
  public static TalonFX ballOutake = new TalonFX(5);
  public Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  public static Solenoid intakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 7);

  public static DigitalInput BeamBreaker1 = new DigitalInput(1);
  public static DigitalInput BeamBreaker2 = new DigitalInput(2);
  public static DigitalInput BeamBreaker3 = new DigitalInput(0);
  public static boolean beamBreaker1, beamBreaker2, beamBreaker3;

  static ColorSensorV3 colorSensor = new ColorSensorV3(Port.kMXP);
  static ColorMatch colorMatcher = new ColorMatch();
  static Color kBlueTarget = new Color(0,0,255);
  static Color kGreenTarget = new Color(0,255,0);
  static Color kRedTarget = new Color(255,0,0);

  int color;

  //PIDController pidController = new PIDController(0, 0, 0, 0);


  public IntakeSubsystem() {
    
    /*BallIntake*/ {
      ballIntake.configFactoryDefault();
      ballIntake.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
      ballIntake.setSensorPhase(true);
      ballIntake.setInverted(false);
      ballIntake.configNominalOutputForward(0, 30);
      ballIntake.configNominalOutputReverse(0, 30);
      ballIntake.configPeakOutputForward(1, 30);
      ballIntake.configPeakOutputReverse(-1, 30);
      ballIntake.configAllowableClosedloopError(0, 0, 30);
      ballIntake.config_kF(0, 0.0, 0);
      ballIntake.config_kP(0, 0.02, 30);
      ballIntake.config_kI(0, 0, 30);
      ballIntake.config_kD(0, 0, 30);
    }

    /*BallOutake*/ {
      ballOutake.configFactoryDefault();
      ballOutake.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
      ballOutake.setSensorPhase(true);
      ballOutake.setInverted(false);
      ballOutake.configNominalOutputForward(0, 30);
      ballOutake.configNominalOutputReverse(0, 30);
      ballOutake.configPeakOutputForward(1, 30);
      ballOutake.configPeakOutputReverse(-1, 30);
      ballOutake.configAllowableClosedloopError(0, 0, 30);
      ballOutake.config_kF(0, 0.0, 0);
      ballOutake.config_kP(0, 0.02, 30);
      ballOutake.config_kI(0, 0, 30);
      ballOutake.config_kD(0, 0, 30);
      ballOutake.configClosedloopRamp(0.5);
      ballOutake.configOpenloopRamp(0.2);
    }

    /*BallFeeder*/ {
      ballFeeder.configFactoryDefault();
      ballFeeder.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
      ballFeeder.setSensorPhase(true);
      ballFeeder.setInverted(false);
      ballFeeder.configNominalOutputForward(0, 30);
      ballFeeder.configNominalOutputReverse(0, 30);
      ballFeeder.configPeakOutputForward(1, 30);
      ballFeeder.configPeakOutputReverse(-1, 30);
      ballFeeder.configAllowableClosedloopError(0, 0, 30);
      ballFeeder.config_kF(0, 0.0, 0);
      ballFeeder.config_kP(0, 0.02, 30);
      ballFeeder.config_kI(0, 0, 30);
      ballFeeder.config_kD(0, 0, 30); 
    }

    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kGreenTarget);
    colorMatcher.addColorMatch(kRedTarget);
  }

  @Override
  public void periodic() {
    {
          // Color detectedColor = colorSensor.getColor();
    // ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    // beamIntake = BeamIntake.get();
    // beamBall1 = BeamBall1.get();
    // beamBall2 = BeamBall2.get();

    // if (match.color == kRedTarget) {
    //   color = 1;
    // } else if (match.color == kBlueTarget) {
    //   color = 2;
    // }

    // if (beamIntake == true && beamBall1 == false && beamBall2 == false) {
    //   ballFeeder.set(TalonFXControlMode.PercentOutput, 0.2);
      
    //   if (match.color == kRedTarget) {
        
    //     ballFeeder.set(TalonFXControlMode.PercentOutput, 0.2);
    //     ballOutake.set(TalonFXControlMode.PercentOutput, -0.9);
        
    //   } else if (match.color == kBlueTarget) {
    //     ballFeeder.set(TalonFXControlMode.PercentOutput, 0.2);
    //     ballOutake.set(TalonFXControlMode.PercentOutput, 0.9);
    //   }
    //   else {
    //     ballFeeder.set(TalonFXControlMode.PercentOutput, 0.2);
    //   }

    // }

    // else if (beamIntake == true && beamBall1 == false && beamBall2 == true) {
    //   if (match.color == kRedTarget) {
    //     ballFeeder.set(TalonFXControlMode.PercentOutput, 0.2);
    //     ballOutake.set(TalonFXControlMode.PercentOutput, -0.9);
    //   } else if (match.color == kBlueTarget) {
    //     ballFeeder.set(TalonFXControlMode.PercentOutput, 0.2);
    //     ballOutake.set(TalonFXControlMode.PercentOutput, 0.9);
    //   }
    //   else {
    //     ballFeeder.set(TalonFXControlMode.PercentOutput, 0.2);
    //   }
    // }

    // else if (beamIntake == false && beamBall1 == true && beamBall2 == true && color == 1) {
    //   ballFeeder.set(TalonFXControlMode.PercentOutput, 0);
    //   ballOutake.set(TalonFXControlMode.PercentOutput, 0);
    // }

    // else if (beamIntake == false && beamBall1 == true && beamBall2 == true && color == 2) {
    //   ballFeeder.set(TalonFXControlMode.PercentOutput, 0.2);
    //   ballOutake.set(TalonFXControlMode.PercentOutput, 0.9);
    // }

    // else if (beamIntake == true && beamBall1 == true && beamBall2 == true) {
    //   ballFeeder.set(TalonFXControlMode.PercentOutput, 0);
    //   ballOutake.set(TalonFXControlMode.PercentOutput, 0);
    // }

    // else if (beamIntake == false && beamBall1 == false && beamBall2 == true) {
    //   ballFeeder.set(TalonFXControlMode.PercentOutput, 0);
    //   ballOutake.set(TalonFXControlMode.PercentOutput, 0);
      // }

    // else if (beamIntake == false && beamBall1 == false && beamBall2 == false) {
    //     ballFeeder.set(TalonFXControlMode.PercentOutput, 0);
    //     ballOutake.set(TalonFXControlMode.PercentOutput, 0);
    //   }
    }

    }

    public void intakeBall(double power) {
      ballIntake.set(ControlMode.PercentOutput, power);
      ballIntake2.set(ControlMode.PercentOutput, -power);
    }

    public boolean detectBeamBreaker1() { 
      beamBreaker1 = BeamBreaker1.get();
      return beamBreaker1;
    }

    public boolean detectBeamBreaker2() { 
      beamBreaker2 = BeamBreaker2.get();
      return beamBreaker2;
    }

    public boolean detectBeamBreaker3() { 
      beamBreaker3 = BeamBreaker3.get();
      return beamBreaker3;
    }

    public void feedBall(){
      ballFeeder.set(ControlMode.PercentOutput, 0.2);
    }

    public static int ballColor;

    public int detectBallColor() {
      Color detectedColor = colorSensor.getColor();
      ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
      Color color = match.color;

      if (color == kRedTarget) {
        SmartDashboard.putString("Ball Color", "Red");
        ballColor = 1;

      }

      else if (color == kBlueTarget) {
        SmartDashboard.putString("Ball Color", "Blue");
        ballColor = 2;
      }
      return ballColor;
    }

    public void uptakeBall(){
      // ballFeeder.set(TalonFXControlMode.PercentOutput, 0.2);
      ballOutake.set(TalonFXControlMode.PercentOutput, -0.5);
      if (detectBeamBreaker3()==true){
        ballOutake.set(TalonFXControlMode.PercentOutput, 0);
      }
    }

    public void outtakeBall(){
      // ballFeeder.set(TalonFXControlMode.PercentOutput, 0.2);
      ballOutake.set(TalonFXControlMode.PercentOutput, 0.5);
      if (detectBeamBreaker2()==false){
        ballOutake.set(TalonFXControlMode.PercentOutput, 0.2);
      }
      
    }

    public void stopIntake(){
      ballIntake.set(ControlMode.PercentOutput, 0);
      ballIntake2.set(ControlMode.PercentOutput, 0);
      ballFeeder.set(ControlMode.PercentOutput, 0);
      ballOutake.set(ControlMode.PercentOutput, 0);
      intakeSolenoid.set(false);
      
    }
  }