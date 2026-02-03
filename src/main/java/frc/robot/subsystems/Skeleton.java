// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SkeletonConstants;
import frc.robot.Constants.SkeletonConstants.SkeletonWantedState;
import frc.robot.Constants.SkeletonConstants.SystemState;
import frc.util.LoggedTunableNumber;

public class Skeleton extends SubsystemBase {
  /* MOTORS */
  private TalonFX skeletonMotor = new TalonFX(SkeletonConstants.skeletonMotorID, "rio");
  private TalonFXConfiguration skeletonMotorConfig = new TalonFXConfiguration();
  //for velocity control
  private double motorspeed = 0.0;
  final MotionMagicVelocityVoltage mm_request = new MotionMagicVelocityVoltage(0);
  //for position control
  private double position = 0.0;
  final MotionMagicExpoVoltage mmE_request = new MotionMagicExpoVoltage(0);

  /* PIDFF CONTROL */
  private LoggedTunableNumber k_S = new LoggedTunableNumber("skeleton_s", SkeletonConstants.skeletonSVA[0]);
  private LoggedTunableNumber k_V = new LoggedTunableNumber("skeleton_v", SkeletonConstants.skeletonSVA[1]);
  private LoggedTunableNumber k_A = new LoggedTunableNumber("skeleton_a", SkeletonConstants.skeletonSVA[2]);
  private LoggedTunableNumber k_P = new LoggedTunableNumber("skeleton_p", SkeletonConstants.skeletonPID[0]);
  private LoggedTunableNumber k_I = new LoggedTunableNumber("skeleton_i", SkeletonConstants.skeletonPID[1]);
  private LoggedTunableNumber k_D = new LoggedTunableNumber("skeleton_d", SkeletonConstants.skeletonPID[2]);

  /* STATES */
  SkeletonWantedState wantedState = SkeletonWantedState.IDLE;
  SystemState systemState = SystemState.IDLING;

  /** Creates a new Skeleton */
  public Skeleton() {
    /* SETUP CONFIG */
    
    // CURRENT LIMITS
    skeletonMotorConfig.CurrentLimits.SupplyCurrentLimit = SkeletonConstants.SupplyCurrentLimit;
    skeletonMotorConfig.CurrentLimits.StatorCurrentLimit = SkeletonConstants.StatorCurrentLimit;
    
    //PID CONSTANTS
    skeletonMotorConfig.Slot0.kS = k_S.get();
    skeletonMotorConfig.Slot0.kV = k_V.get();
    skeletonMotorConfig.Slot0.kA = k_A.get(); 
    skeletonMotorConfig.Slot0.kP = k_P.get();
    skeletonMotorConfig.Slot0.kI = k_I.get();
    skeletonMotorConfig.Slot0.kD = k_D.get();

    //use this for velocity motion magic 
    skeletonMotorConfig.MotionMagic.MotionMagicAcceleration = SkeletonConstants.skeletonMotionMagicAccel; // Target acceleration of 160 rps/s (0.5 seconds)
    skeletonMotorConfig.MotionMagic.MotionMagicJerk = SkeletonConstants.skeletonMotionMagicJerk;

    //use this for motion magic expo (very good control of position)
    skeletonMotorConfig.MotionMagic.MotionMagicExpo_kV = SkeletonConstants.skeletonMotionMagicExpoK_V;
    skeletonMotorConfig.MotionMagic.MotionMagicExpo_kA = SkeletonConstants.skeletonMotionMagicExpoK_A;

    //APPLY CONFIG TO MOTOR
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = skeletonMotor.getConfigurator().apply(skeletonMotorConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

  public void setWantedSkeletonMode(SkeletonWantedState desiredState) {
    this.wantedState = desiredState;
  }

  private SystemState changeCurrentSystemState() {
    return switch (wantedState) {
      case IDLE: 
        yield SystemState.IDLING;
    };
  }

    
  private void applyState(){
    switch(systemState){
      case IDLING:
        motorspeed = 0.0;
        position = 0.0;
        break;
    }
  }


  /**
   * Check LoggedTunableNumbers. If changed, update PID and SVA values of motor
   */
  public void checkTunableValues() {
    if (k_S.hasChanged() || k_V.hasChanged() || k_A.hasChanged() 
    || k_P.hasChanged() || k_I.hasChanged() || k_D.hasChanged()) {
      skeletonMotorConfig.Slot0.kS = k_S.get();
      skeletonMotorConfig.Slot0.kV = k_V.get();
      skeletonMotorConfig.Slot0.kA = k_A.get(); 
      skeletonMotorConfig.Slot0.kP = k_P.get();
      skeletonMotorConfig.Slot0.kI = k_I.get();
      skeletonMotorConfig.Slot0.kD = k_D.get();
    }
    skeletonMotor.getConfigurator().apply(skeletonMotorConfig);
  }

  @Override
  public void periodic() {
    checkTunableValues();
    systemState = changeCurrentSystemState();
    applyState();
    //example of how to control motor for velocity
    skeletonMotor.setControl(mm_request.withVelocity(motorspeed));
    //example of how to control motor for position
    skeletonMotor.setControl(mmE_request.withPosition(position));
  }

}
