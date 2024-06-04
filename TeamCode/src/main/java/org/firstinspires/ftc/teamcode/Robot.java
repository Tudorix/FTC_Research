package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {

    private static Robot robot = null;
    
    //Drivetrain
    public IMU imu = null;
    public IMU.Parameters parameters = null;
    public final DcMotorEx FL, FR, BL, BR;
    public final double MAX_POWER_DRIVETRAIN = 0.85;
    
    //Sweeper
    public DcMotor SweeperMotor = null;
    public double MAX_POWER_SWEEP = 0.8;
    
    //Slides
    public final DcMotor LS , RS;
    public final double MAX_POWER_SLIDES = 0.85;
    public final double SLIDES_EQUILIBRIUM = 0.25;
    
    //Outtake
    public Servo rotateArm = null;
    public Servo stopper = null;
    
    public final double ROTATEARM_PLACE_POSITION = 0.4;
    public final double ROTATEARM_TAKE_POSITION = 1;
    
    public final double STOPPER_OPEN = 0.8;
    public final double STOPPER_CLOSED = 1;
    
    //Airplane
    public Servo plane = null;
    
    //HardwareMap
    
    public HardwareMap hardwareMap = null;
    
    private Robot(HardwareMap hwMap) {
        this.FL = hwMap.get(DcMotorEx.class, "FL");
        this.FR = hwMap.get(DcMotorEx.class, "FR");
        this.BL = hwMap.get(DcMotorEx.class, "BL");
        this.BR = hwMap.get(DcMotorEx.class, "BR");

        this.LS = hwMap.get(DcMotor.class, "LS");
        this.RS = hwMap.get(DcMotor.class, "RS");
        
        this.hardwareMap = hwMap;
        
        this.SweeperMotor = hwMap.get(DcMotor.class, "A");
        
        this.rotateArm = hwMap.get(Servo.class , "ARM");
        this.stopper = hwMap.get(Servo.class , "STOP");
        this.plane = hwMap.get(Servo.class , "P");

        imu = hwMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
    }
    
    public static synchronized Robot getInstance(HardwareMap hwMap){
        if(robot == null)
            robot = new Robot(hwMap);
        return robot;
    }






}
