package org.firstinspires.ftc.teamcode.old;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TurnOverMotorOuttakeSystem {
    /*private static TurnOverMotorOuttakeSystem single_instance = null;
    private boolean running = false;
    Thread OuttakeSystem_FallingPixels = null;
    
    DcMotorEx TurnOverMotor;
    Gamepad gm2;
    double  MaxPower;
    
    public TurnOverMotorOuttakeSystem(Robot instance , Gamepad gm2){
        this.TurnOverMotor = instance.turnOverMotor;
        this.gm2 = gm2;
        MaxPower = instance.MAX_POWER;
    }
    
    public void start(){
        running = true;
        if(OuttakeSystem_FallingPixels == null || !OuttakeSystem_FallingPixels.isAlive()){
            OuttakeSystem_FallingPixels = new Thread(() -> {
                while(running){
                    if(gm2.dpad_up){
                        rotateToOuttake();
                    }else if(gm2.dpad_down){
                        rotateToIntake();
                    }else{
                        stopMotor();
                    }
                }
            });
        }
        OuttakeSystem_FallingPixels.start();
    }

    public void rotateToOuttake(){TurnOverMotor.setPower(MaxPower * 0.5);}
    public void rotateToIntake(){TurnOverMotor.setPower(-MaxPower * 0.5);}
    public void stopMotor(){TurnOverMotor.setPower(0);}

    public void stop(){
        running = false;
    }
    
    public boolean getStatus(){
        return running;
    }
    
    public static synchronized TurnOverMotorOuttakeSystem getInstance(HardwareMap hMap, Gamepad gm){
        if(single_instance == null){
            single_instance = new TurnOverMotorOuttakeSystem(Robot.getInstance(hMap), gm);
        }
        return single_instance;
    }
     */
}
