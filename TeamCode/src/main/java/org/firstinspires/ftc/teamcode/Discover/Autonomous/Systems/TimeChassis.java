package org.firstinspires.ftc.teamcode.Discover.Autonomous.Systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Discover.HardwareClass;

public class TimeChassis {
    
    public DcMotor FL = null;
    public DcMotor FR = null;
    public DcMotor BL = null;
    public DcMotor BR = null;
    
    double x = 0 , cos = 0 , sin = 0;
    double y = 0 , max=0 , FLPower = 0 , FRPower = 0, BLPower = 0 , BRPower = 0;
    double turn = 0 , theta = 0 , power = 0;
    
    public TimeChassis(HardwareClass hardwareClass){
        this.FL = null;
        this.FR = null;
        this.BR = null;
        this.BL = null;
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
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
