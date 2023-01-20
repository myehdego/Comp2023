// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CANIDs;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  CANSparkMax retractorMotor;
  CANSparkMax raiserMotor;
  private RelativeEncoder retractorEncoder;
  private RelativeEncoder raiserEncoder;
  
  /** The arm will be able to have a range of motion cosisting of going up, down, extend, 
   * and retract to have the ability to reach futher up on the shelves and pegs  */
  public Arm() {
    retractorMotor = new CANSparkMax(CANIDs.ArmRetractorMotor, MotorType.kBrushless);
    retractorMotor.setInverted(CANIDs.retractorMotorInverted);
    // raiserMotor = new CANSparkMax(CANIDs.ArmRaiserMotor, MotorType.kBrushless);
    // raiserMotor.setInverted(CANIDs.ArmRaiserMotorInverted);
    retractorEncoder = retractorMotor.getEncoder();
    // raiserEncoder = raiserMotor.getEncoder();
  }
  
  /* public void raise() {
    raiserMotor.set(0.1);
  }
  public void raise(boolean direction) {
    if(direction) raise();
    else lower();
  }

  public void lower() {
    raiserMotor.set(-0.1);
  } */

  public void extend() {
    retractorMotor.set(0.3);
  }
  public void extend(boolean direction) {
    if(direction) extend();
    else retract();
  }

  public void retract() {
    retractorMotor.set(-0.3);
  }

  public void stopRaise() {
    raiserMotor.stopMotor();
  }

  public void stopExtend() {
    retractorMotor.stopMotor();
  }

  public double getExtenderPos() {
    return retractorEncoder.getPosition();
  }

  public void resetEncoders() {
    retractorEncoder.setPosition(0);
    // raiserEncoder.setPosition(0);
  }

  public CommandBase extensionCommand(double Target) {
    return runOnce(() -> {extend(retractorEncoder.getPosition() < Target);
                         SmartDashboard.putNumber("target", Target);}) 

      .andThen(() -> {
        //extend();
        SmartDashboard.putBoolean("currentPosTest", retractorEncoder.getPosition() >= Target - ArmConstants.retractorTolerance 
        && retractorEncoder.getPosition() <= Target + ArmConstants.retractorTolerance);
      })

      .until(() -> (retractorEncoder.getPosition() >= Target - ArmConstants.retractorTolerance
                    && retractorEncoder.getPosition() <= Target + ArmConstants.retractorTolerance))

     // .andThen(() -> stopExtend())

      .finallyDo(interrupt -> stopExtend())
      ;
  }

  /* public CommandBase raisingComand(double Target) {
    return runOnce(() -> raise(raiserEncoder.getPosition() < Target))

    .until(() -> raiserEncoder.getPosition() >= Target - ArmConstants.raiserTolerance 
              && raiserEncoder.getPosition() <= Target + ArmConstants.raiserTolerance)

    .finallyDo(interrupt -> stopRaise())
    ;
  } */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm extender position", retractorEncoder.getPosition());
  }
}
