package org.firstinspires.ftc.teamcode.old;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;

@Config
public class IntakeSystem_Sweeper {
    private static IntakeSystem_Sweeper single_instance = null;
    private boolean running = false;
    Thread IntakeSystem_Sweeper = null;
    DcMotorEx sweepMotor;
    Gamepad gm2;
    int HoldUp , HoldDown;
    double  MaxPower;
    boolean isOn = false;
    
    public IntakeSystem_Sweeper(Robot instance , Gamepad gm2){
        //this.sweepMotor = instance.sweepMotor;
        //this.gm2 = gm2;
        //MaxPower = instance.MAX_POWER;
    }
    
    public void start(){
        running = true;
        if(IntakeSystem_Sweeper == null || !IntakeSystem_Sweeper.isAlive()){
            IntakeSystem_Sweeper = new Thread(() -> {
                while(running){
                    if(gm2.y) isOn = true;
                    if(gm2.b) isOn = false;
                    
                    if(isOn) sweepMotor.setPower(1 * (gm2.right_trigger-gm2.left_trigger));
                    if(!isOn) sweepMotor.setPower(0);
                }
            });
        }
        IntakeSystem_Sweeper.start();
    }
    
    public void stop(){
        running = false;
    }
    
    public boolean getStatus(){
        return running;
    }
    
    public static synchronized IntakeSystem_Sweeper getInstance(HardwareMap hMap, Gamepad gm){
        if(single_instance == null){
            single_instance = new IntakeSystem_Sweeper(Robot.getInstance(hMap), gm);
        }
        return single_instance;
    }
}
