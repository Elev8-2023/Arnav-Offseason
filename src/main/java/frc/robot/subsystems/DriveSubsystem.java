package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.MotControllerJNI;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {


  private final CANSparkMax L1;
  private final CANSparkMax L2;
  private final CANSparkMax R1;
  private final CANSparkMax R2;

  private final MotorControllerGroup Left;
  private final MotorControllerGroup Right;

  private static DifferentialDrive Drive1;

  public DriveSubsystem() {

    L1 = new CANSparkMax(2, MotorType.kBrushless);
    L2 = new CANSparkMax(1, MotorType.kBrushless);
    R1 = new CANSparkMax(3, MotorType.kBrushless);
    R2 = new CANSparkMax(11, MotorType.kBrushless);

    Left = new MotorControllerGroup(L1, L2); Left.setInverted(true);
    Right = new MotorControllerGroup(R1, R2); 

    Drive1 = new DifferentialDrive(Left, Right);

  }

  @Override
  public void periodic() {

  }

  public static void drive(double xSpeed, double zRotation) {
    Drive1.arcadeDrive(0.5*xSpeed, 0.5*zRotation);
  }
}
