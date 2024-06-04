package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;

@Config
public class SlidesSystem {
    private static SlidesSystem single_instance = null;
    private boolean running = false;
    Thread SlidesSystem = null;

    DcMotor LeftSlide , RightSlide;
    Gamepad gm;
    double  MaxPower , Equal;

    public SlidesSystem(Robot instance , Gamepad gm){
        this.LeftSlide = instance.LS;
        this.RightSlide = instance.RS;
        this.gm = gm;
        this.MaxPower = instance.MAX_POWER_SLIDES;
        this.Equal = instance.SLIDES_EQUILIBRIUM;
    }
    
    public void start(){
        running = true;
        if(SlidesSystem == null || !SlidesSystem.isAlive()){
            SlidesSystem = new Thread(() -> {
                while(running){
                    if(gm.right_trigger > 0.5){
                        LeftSlide.setPower(-MaxPower * gm.right_trigger);
                        RightSlide.setPower(MaxPower * gm.right_trigger);
                    }else if(gm.left_trigger > 0.5){
                        LeftSlide.setPower(MaxPower * gm.left_trigger);
                        RightSlide.setPower(-MaxPower * gm.left_trigger);
                    }else if(gm.right_bumper){
                        LeftSlide.setPower(Equal);
                        RightSlide.setPower(Equal);
                    }else{
                        LeftSlide.setPower(0);
                        RightSlide.setPower(0);
                    }
                }
            });
        }
        SlidesSystem.start();
    }
    
    public void stop(){
        running = false;
    }
    
    public boolean getStatus(){
        return running;
    }
    
    public static synchronized SlidesSystem getInstance(HardwareMap hMap, Gamepad gm){
        if(single_instance == null){
            single_instance = new SlidesSystem(Robot.getInstance(hMap), gm);
        }
        return single_instance;
    }
}
