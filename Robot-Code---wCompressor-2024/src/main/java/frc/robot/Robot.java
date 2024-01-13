// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private double m_leftStick;
  private double m_rightStick;
  private Trigger m_rightTrigger;
  private Trigger m_leftTrigger;
  private double m_leftPos;
  private double m_rightPos;
  private double speedInhibitor;
  private double steeringInhibitor;
  CommandXboxController controller;
  

 


  private final MotorController leftMaster = new CANSparkMax(1, MotorType.kBrushless);
  private final MotorController leftSlave = new CANSparkMax(2, MotorType.kBrushless);
  private final MotorController rightMaster = new CANSparkMax(3, MotorType.kBrushless);
  private final MotorController rightSlave = new CANSparkMax(4, MotorType.kBrushless);

  MotorControllerGroup rightGroup = new MotorControllerGroup(rightMaster, rightSlave);
  MotorControllerGroup leftGroup = new MotorControllerGroup(leftMaster, leftSlave); 

  Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

 // DifferentialDrive difDrive = new DifferentialDrive(leftGroup, rightGroup);










   

  @Override



  
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    rightGroup.setInverted(true);
    controller = new CommandXboxController(0);
    

    speedInhibitor = 0.5;
    steeringInhibitor = 0.5;
    
    

    pcmCompressor.enableDigital();
    pcmCompressor.disable();

    
    boolean pressureSwitch = pcmCompressor.getPressureSwitchValue();
    double current = pcmCompressor.getCurrent();
    
    pcmCompressor.enableDigital();

  }

  @Override
  public void teleopPeriodic() {
    boolean useTrigger = true;
    //difDrive.tankDrive(-m_leftStick.getY(), -m_rightStick.getY());
    if (useTrigger) {

    double m_leftStick = controller.getLeftX();
    double m_rightStick =  controller.getRightY();
  
    m_leftPos = controller.getLeftTriggerAxis();
    m_rightPos = controller.getRightTriggerAxis();

    double speed = MathUtil.applyDeadband(m_rightPos - m_leftPos, 0.1);
    double steering = MathUtil.applyDeadband(m_leftStick, 0.1);
    //double joydrive = MathUtil.applyDeadband(m_rightStick, 0.1);

    speed *= speedInhibitor;
    steering *= steeringInhibitor;
   // joydrive *= speedInhibitor;

    double leftSpeed = speed - steering;
    double rightSpeed = speed + steering;
   // double joySpeed = joydrive;

    leftSpeed = MathUtil.clamp(leftSpeed, -1, 1);
    rightSpeed = MathUtil.clamp(rightSpeed, -1, 1);

   
    leftMaster.set(leftSpeed);
    leftSlave.set(leftSpeed);

    rightMaster.set(rightSpeed);
    rightSlave.set(rightSpeed);
    }

  }
}

  

  



  
