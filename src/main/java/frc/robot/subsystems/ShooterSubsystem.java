package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  public final TalonFX shooterFlywheel = new TalonFX(Constants.shooterPort);
  public static final TalonFX shooterHood = new TalonFX(Constants.hoodPort);
  public final TalonFX loadBall = new TalonFX(Constants.loadBallPort);
  public static final TalonFX shooterTurret = new TalonFX(Constants.turretPort);

  public double shooterRPM, t_rpm, hood_ogpos, tx, tError, ty, dist;
  public double tPrevError = 0.0;


  public ShooterSubsystem() {

    /*Shooter Flywheel*/{
      shooterFlywheel.configFactoryDefault();
      shooterFlywheel.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
      shooterFlywheel.setSensorPhase(true);
      shooterFlywheel.setInverted(false);
      shooterFlywheel.configNominalOutputForward(0, 30);
      shooterFlywheel.configNominalOutputReverse(0, 30);
      shooterFlywheel.configPeakOutputForward(1, 30);
      shooterFlywheel.configPeakOutputReverse(-1, 30);
      shooterFlywheel.configAllowableClosedloopError(0, 0, 30);
      shooterFlywheel.config_kP(0, 0.38, 30);
      shooterFlywheel.config_kD(0, 10.0, 30);
      shooterFlywheel.config_kI(0, 0.0, 30);
      shooterFlywheel.config_kF(0, 0.0673, 30);
      shooterFlywheel.configClosedloopRamp(1);
    }
      
    /*Shooter Hood*/ {
      shooterHood.configFactoryDefault();
      shooterHood.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
      shooterHood.setSensorPhase(true);
      shooterHood.setInverted(false);
      shooterHood.configNominalOutputForward(0, 30);
      shooterHood.configNominalOutputReverse(0, 30);
      shooterHood.configPeakOutputForward(1, 30);
      shooterHood.configPeakOutputReverse(-1, 30);
      shooterHood.configAllowableClosedloopError(0, 150.0, 30);
      shooterHood.config_kP(0, 0.15, 30);
      shooterHood.config_kD(0, 0.0005, 30);
      shooterHood.config_kI(0, 0.001, 30);
      shooterHood.config_kF(0, 0.0, 30);
      shooterHood.configClosedloopRamp(0);
    }

    /*Shooter Turret*/{
      shooterTurret.configFactoryDefault();
      shooterTurret.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
      shooterTurret.setSensorPhase(true);
      shooterTurret.setInverted(false);
      shooterTurret.configNominalOutputForward(0, 30);
      shooterTurret.configNominalOutputReverse(0, 30);
      shooterTurret.configPeakOutputForward(1, 30);
      shooterTurret.configPeakOutputReverse(-1, 30);
      shooterTurret.configAllowableClosedloopError(0, 0, 30);
      shooterTurret.config_kP(0, 0.4, 30);
      shooterTurret.config_kD(0, 8.0, 30);
      shooterTurret.config_kI(0, 0.0003, 30);
      shooterTurret.config_kF(0, 0.0, 30);
      shooterTurret.configClosedloopRamp(0);
    }
    
  }

  @Override
  public void periodic() {
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    turretAction();
    dist = Constants.goalht/Math.tan(Math.toRadians(ty+Constants.limeangle));
    SmartDashboard.putNumber("Dist", dist);
    SmartDashboard.putNumber("hAng", getHAngDemand(dist));
    hoodAngle(getHAngDemand(dist));
    shooterRPM = (600*shooterFlywheel.getSelectedSensorVelocity())/2048;
    //t_rpm = 600/2048*turret.getSelectedSensorVelocity();
    hood_ogpos = shooterHood.getSelectedSensorPosition()/Constants.hoodReduction;
    SmartDashboard.putNumber("shooter rpm", shooterRPM); //may want to add gear reductions
    SmartDashboard.putNumber("turret rpm", t_rpm); //may want to add gear reductions
    SmartDashboard.putNumber("Hood Position Degrees", hood_ogpos);
    SmartDashboard.putNumber("Hood Position Sensor Positions", shooterHood.getSelectedSensorPosition());
    SmartDashboard.putNumber("Hood Setpoint Sensor Positions", 20*Constants.hoodReduction);
    SmartDashboard.putNumber("tx", tx);
    SmartDashboard.putNumber("TPos", shooterTurret.getSelectedSensorPosition()/Constants.turretReduction);
    // This method will be called once per scheduler run
  }

  public void shootRPM(double rpm) {
    shooterFlywheel.set(TalonFXControlMode.Velocity, (rpm*2048)/600);
    
    if (shooterRPM >= .95*rpm)
    {
      loadBall.set(TalonFXControlMode.PercentOutput, 0.5);
    }
  }

  public void turretAction()
  {
    //turret.set(TalonFXControlMode.PercentOutput, pow);
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    
    tError = shooterTurret.getSelectedSensorPosition()+(tx*Constants.turretReduction);
    
    if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0)==1.0)
    {
      if (Math.abs(tError) < 9600)
      {
      shooterTurret.set(TalonFXControlMode.Position, tError);
      }
      else
      {
        shooterTurret.set(TalonFXControlMode.Position, 9600*Math.signum(tError));
      }
    }
    else
    {
      tError = tPrevError;
      shooterTurret.set(TalonFXControlMode.Position, tError);
    }
    tPrevError = tError;
  }

  public void turret2ang (double ang)
  {
    shooterTurret.set(TalonFXControlMode.Position, (ang*Constants.turretReduction));

  }

  public static void hoodAngle(double angle)
  {
    //pass
    if (angle>=40)
    {angle = 40;}
    if (angle<=0)
    {angle = 0;}

    shooterHood.set(TalonFXControlMode.Position, angle*Constants.hoodReduction);
  }

  public double getPowDemand(double dist)
  {
    return (-143.362*dist + 47.619*dist*dist + 2.0202*dist*dist*dist + 2295.24);
  }

  public double getHAngDemand (double dist)
  {
    return (9.95815*dist-2.04762*dist*dist+0.2626*dist*dist*dist-5.14286);
  }

}
