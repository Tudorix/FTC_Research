package org.firstinspires.ftc.teamcode.Threads.Drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Systems.DistanceSystem;

public class HolonomicDrive {
    
    DcMotor turnOverMotor = null;
    Gamepad gm;
    
    double  maxPower;
    private static HolonomicDrive single_instance = null;
    private boolean running = false;
    private DcMotor FR , FL , BR , BL;
    private double BLPower , BRPower , FRPower , FLPower;
    private double power , turn , max , x , y;
    private double sin , cos , theta;
    Thread DriveTrain = null;
    public double orientation;
    public boolean mode = true , PID = false ;
    
    DistanceSystem distanceSystem = null;
    
    public HolonomicDrive(Robot instance, Gamepad gm){
        this.gm = gm;
        this.maxPower = instance.MAX_POWER_DRIVETRAIN;
        this.FR = instance.FR;
        this.FL = instance.FL;
        this.BR = instance.BR;
        this.BL = instance.BL;
        distanceSystem = DistanceSystem.getInstance(instance.hardwareMap);
        
    }
    
    public void start(){
        running=true;
        orientation = 0;
        if(DriveTrain == null || !DriveTrain.isAlive()){
            DriveTrain = new Thread(() ->{
                while(running){
                    x = gm.left_stick_x;
                    y = -gm.left_stick_y;
                    turn = gm.right_stick_x;
                    
                    theta = Math.atan2(y , x);
                    power = Math.hypot(x, y);

                    sin = Math.sin(theta - Math.PI/4);
                    cos = Math.cos(theta - Math.PI/4);
                    max = Math.max(Math.abs(sin) , Math.abs(cos));

                    FLPower = power * cos/max + turn;
                    FRPower = power * sin/max - turn;
                    BLPower = power * sin/max + turn;
                    BRPower = power * cos/max - turn;

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
                    
                    
                }
            });
        }
        DriveTrain.start();
    }
    
    public void stop(){
        running = false;
    }
    
    public boolean getStatus(){
        return running;
    }
    
    public static synchronized HolonomicDrive getInstance(HardwareMap hMap, Gamepad gm){
        if(single_instance == null){
            single_instance = new HolonomicDrive(Robot.getInstance(hMap), gm);
        }
        return single_instance;
    }
}
