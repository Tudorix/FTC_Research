package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;

public class DriveTrain {
    
    DcMotor turnOverMotor = null;
    Gamepad gm1;
    
    double  maxPower;
    private static DriveTrain single_instance = null;
    private boolean running = false;
    private DcMotor FR , FL , BR , BL;
    private double BLPower , BRPower , FRPower , FLPower;
    private double power , turn , max , x , y;
    private double sin , cos , theta;
    Thread DriveTrain = null;
    
    public DriveTrain(Robot instance, Gamepad gm1){
        this.gm1 = gm1;
        this.maxPower = instance.MAX_POWER_DRIVETRAIN;
        this.FR = instance.FR;
        this.FL = instance.FL;
        this.BR = instance.BR;
        this.BL = instance.BL;

        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    
    public void start(){
        running=true;
        if(DriveTrain == null || !DriveTrain.isAlive()){
            DriveTrain = new Thread(() ->{
                while(running){
                    //Mecanum Drive
                    x = gm1.left_stick_x;
                    y = -gm1.left_stick_y;
                    turn = gm1.right_stick_x ;

                    //Multiplier = (-x + y)/5;
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
    
    public static synchronized DriveTrain getInstance(HardwareMap hMap, Gamepad gm){
        if(single_instance == null){
            single_instance = new DriveTrain(Robot.getInstance(hMap), gm);
        }
        return single_instance;
    }
}
