package org.firstinspires.ftc.teamcode.Threads.Drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Systems.DistanceSystem;

public class StataicHeading_Drive {
    
    Gamepad gm;
    
    private static StataicHeading_Drive single_instance = null;
    private boolean running = false;
    private DcMotor FR , FL , BR , BL;
    private double BLPower , BRPower , FRPower , FLPower;
    private double power , turn , max , x , y;
    private double sin , cos , theta;
    Thread staticHeading_Drive = null;
    public double orientation;
    
    public StataicHeading_Drive(Robot instance, Gamepad gm){
        this.gm = gm;
        this.FR = instance.FR;
        this.FL = instance.FL;
        this.BR = instance.BR;
        this.BL = instance.BL;
        
    }
    
    public void start(){
        running=true;
        orientation = 0;
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        if(staticHeading_Drive == null || !staticHeading_Drive.isAlive()){
            staticHeading_Drive = new Thread(() ->{
                while(running){
                    x = gm.left_stick_y;
                    y = gm.left_stick_x;
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

                    FL.setPower(FLPower - orientation);
                    FR.setPower(FRPower + orientation);
                    BL.setPower(BLPower - orientation);
                    BR.setPower(BRPower + orientation);
                }
            });
        }
        staticHeading_Drive.start();
    }
    
    public void stop(){
        running = false;
    }
    
    public boolean getStatus(){
        return running;
    }
    
    public static synchronized StataicHeading_Drive getInstance(HardwareMap hMap, Gamepad gm){
        if(single_instance == null){
            single_instance = new StataicHeading_Drive(Robot.getInstance(hMap), gm);
        }
        return single_instance;
    }
}
