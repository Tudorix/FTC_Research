package org.firstinspires.ftc.teamcode.BRI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.tensorflow.lite.task.core.vision.ImageProcessingOptions;

import com.arcrobotics.ftclib.controller.PIDController;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.sql.Time;

public class Hardware_BRI {
    // This is a class to declare the hardware components for all other OpModes
    
    
    //Declarations
    public DcMotor FL;
    public DcMotor FR;
    public DcMotor BL;
    public DcMotor BR;
    
    public DcMotor LiftRight;
    double LiftRight_Integral;
    double LiftRight_Repetitions;
    public DcMotor LiftLeft;
    double LiftLeft_Integral;
    double LiftLeft_Repetitions;
    
    public DcMotor TurnOverArm;
    double TurnOverArm_Integral;
    double TurnOverArm_Repetitions;
    
    public Servo Claw;
    public Servo Claw_Rotation;
    
    public BNO055IMU imu;
    public Orientation angles;
    
    //Getting a HardwareMap Component
    HardwareMap hmMap;
    
    //Building a Constructor
    public Hardware_BRI(){}
    
    public void init(HardwareMap hmMap){
        FL = hmMap.dcMotor.get("FL");
        FR = hmMap.dcMotor.get("FR");
        BL = hmMap.dcMotor.get("BL");
        BR = hmMap.dcMotor.get("BR");
        TurnOverArm = hmMap.dcMotor.get("________NUME________");
        LiftRight = hmMap.dcMotor.get("________NUME________");
        LiftLeft = hmMap.dcMotor.get("________NUME________");
        
        Claw = hmMap.servo.get("________NUME________");
        Claw_Rotation = hmMap.servo.get("________NUME________");
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hmMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TurnOverArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
}

