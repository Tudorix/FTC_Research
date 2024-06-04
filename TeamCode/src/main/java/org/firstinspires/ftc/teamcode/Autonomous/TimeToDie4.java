package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Vision.TSESystem;

@Autonomous(name = "BlueClose")
public class TimeToDie4 extends LinearOpMode {
    Robot robot = null;
    TSESystem tseDetection = null;
    int detectionCase = 0;
    
    DcMotor FR, FL , BR, BL;
    private double BLPower , BRPower , FRPower , FLPower;
    private double power , turn , max , x , y;
    private double sin , cos , theta;
    
    public static double driveOne, driveToq;
    
    @Override
    public void runOpMode() throws InterruptedException {
        robot = Robot.getInstance(hardwareMap);
        
        tseDetection = TSESystem.getInstance(hardwareMap);
        
        this.FR = robot.FR;
        this.FL = robot.FL;
        this.BR = robot.BR;
        this.BL = robot.BL;
        
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        
        tseDetection.start("blue");
        while(!isStarted()) {
            detectionCase = tseDetection.SeeCase();
            telemetry.addData("Case", detectionCase);
            telemetry.update();
        }
        waitForStart();
        
        
        if(isStopRequested()) {return;}
    
        Drive(0 , 0.4, 200);
        Drive(-0.4 , 0, 2500);
    }
    
    public void Drive(double XInput, double YInput, long Time){
        
        //Multiplier = (-x + y)/5;
        theta = Math.atan2(YInput , XInput);
        power = Math.hypot(XInput, YInput);
        
        sin = Math.sin(theta - Math.PI/4);
        cos = Math.cos(theta - Math.PI/4);
        max = Math.max(Math.abs(sin) , Math.abs(cos));
        
        FLPower = power * cos/max;
        FRPower = power * sin/max;
        BLPower = power * sin/max;
        BRPower = power * cos/max;
        
        if((power + Math.abs(turn)) > 1){
            FLPower /= power + turn;
            FRPower /= power + turn;
            BLPower /= power + turn;
            BRPower /= power + turn;
        }
        
        
        FL.setPower(FLPower);
        FR.setPower(FRPower);
        BL.setPower(BLPower);
        BR.setPower(BRPower);
        
        try
        {
            Thread.sleep(Time);
        }
        catch(InterruptedException e){}
        
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        
    }
    
    public void Turn(double turn,long Time){
        FLPower = turn;
        FRPower = -turn;
        BLPower = turn;
        BRPower = -turn;
        
        FL.setPower(FLPower);
        FR.setPower(FRPower);
        BL.setPower(BLPower);
        BR.setPower(BRPower);
        
        try
        {
            Thread.sleep(Time);
        }
        catch(InterruptedException e){}
        
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }
    
    
}
