// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The Shooter subsystem controls a shooter mechanism on the robot.
 * Two NEO motors spin in opposite directions (CW and CCW) to shoot game pieces.
 * Adjust motor CAN IDs and speeds to match your robot's hardware.
 */
public class Shooter extends SubsystemBase {

  // TODO: Update CAN IDs to match your robot's wiring
  private final SparkMax shooterMotorTop_cw;
  private final SparkMax shooterMotorTop_ccw;
  private final TalonFX shooterMotorBottom_cw;
  private final SparkMax shooterMotorBottom_ccw_x60;

  /** Creates a new Shooter subsystem. */
  public Shooter() {
    shooterMotorTop_cw = new SparkMax(10, MotorType.kBrushless);         // TODO: Set correct CAN ID
    shooterMotorTop_ccw = new SparkMax(11, MotorType.kBrushless);        // TODO: Set correct CAN ID
    shooterMotorBottom_cw = new TalonFX(12);                             // TODO: Set correct CAN ID (Kraken X60)
    shooterMotorBottom_ccw_x60 = new SparkMax(13, MotorType.kBrushless); // TODO: Set correct CAN ID
  }

  /**
   * Runs all shooter motors at the given speed.
   * CW motors spin in the positive direction, CCW motors spin inverted.
   *
   * @param speed Motor output value, from -1.0 to 1.0.
   */
  public void setSpeed(double speed) {
    shooterMotorTop_cw.set(speed);
    shooterMotorBottom_cw.set(speed);       // Same direction as top CW
    shooterMotorTop_ccw.set(-speed);        // Inverted to spin opposite direction
    shooterMotorBottom_ccw_x60.set(-speed);     // Same direction as top CCW
  }

  /** Stops all shooter motors. */
  public void stop() {
    shooterMotorTop_cw.set(0);
    shooterMotorTop_ccw.set(0);
    shooterMotorBottom_cw.set(0);
    shooterMotorBottom_ccw_x60.set(0);
  }

  /**
   * Returns a command that runs the shooter at the specified speed until interrupted,
   * then stops the motor.
   *
   * @param speed Motor output value, from -1.0 to 1.0.
   * @return A command to run the shooter.
   */
  public Command runShooterCommand(double speed) {
    return this.startEnd(() -> setSpeed(speed), this::stop);
  }

  @Override
  public void periodic() {
    // Publish shooter telemetry to SmartDashboard
    SmartDashboard.putNumber("Shooter Top CW Speed", shooterMotorTop_cw.get());
    SmartDashboard.putNumber("Shooter Top CCW Speed", shooterMotorTop_ccw.get());
    SmartDashboard.putNumber("Shooter Bottom CW Speed", shooterMotorBottom_cw.get());
    SmartDashboard.putNumber("Shooter Bottom CCW Speed", shooterMotorBottom_ccw_x60.get());
  }
}
