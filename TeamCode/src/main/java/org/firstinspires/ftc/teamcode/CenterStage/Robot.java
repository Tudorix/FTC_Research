package org.firstinspires.ftc.teamcode.CenterStage;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {

    private static Robot robot = null;
    
    //Sensors
    public DistanceSensor stopperSensor = null;
    public double minDistanceToBackboardCM = 45; //TO BE CHANGED
    
    /**
     *  SLIDES PID:
     *
     *  P: 0.0008
     *  I: 0
     *  D: 0.000045
     */
    
    
    //Drivetrain
    public final DcMotorEx FL, FR, BL, BR;
    public final double MAX_POWER_DRIVETRAIN = 0.85;
    
    //Sweeper
    public DcMotor SweeperMotor = null;
    public double MAX_POWER_SWEEP = 0.8;
    
    //Slides
    public final DcMotorEx LS , RS;
    
    public final  double kP = 0.001, kI = 0, kD = 0;
    public final double MAX_POWER_SLIDES = 0.75;
    public final double SLIDES_EQUILIBRIUM = 0.04;
    
    //Outtake
    public Servo rotateArm = null;
    public Servo stopper = null;
    public Servo sweepMove = null;

    public RevBlinkinLedDriver blinkingDrive = null;
    
    //Airplane
    public Servo plane = null;
    
    //HardwareMap
    
    public HardwareMap hardwareMap = null;
    
    private Robot(HardwareMap hwMap) {
        this.FL = hwMap.get(DcMotorEx.class, "FL");
        this.FR = hwMap.get(DcMotorEx.class, "FR");
        this.BL = hwMap.get(DcMotorEx.class, "BL");
        this.BR = hwMap.get(DcMotorEx.class, "BR");

        this.LS = hwMap.get(DcMotorEx.class, "LS");
        this.RS = hwMap.get(DcMotorEx.class, "RS");
        
        this.hardwareMap = hwMap;
        
        this.SweeperMotor = hwMap.get(DcMotor.class, "A");
        
        this.rotateArm = hwMap.get(Servo.class , "ARM");
        this.stopper = hwMap.get(Servo.class , "STOP");
        this.plane = hwMap.get(Servo.class , "P");
        this.sweepMove = hwMap.get(Servo.class , "SM");
        
        this.stopperSensor = hwMap.get(DistanceSensor.class , "BS");

        this.blinkingDrive = hwMap.get(RevBlinkinLedDriver.class, "LED");
    }
    
    public static synchronized Robot getInstance(HardwareMap hwMap){
        if(robot == null)
            robot = new Robot(hwMap);
        return robot;
    }






}
