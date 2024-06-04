package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoGotToPositionSystem {

    DcMotor turnOverMotor = null;
    Gamepad gm1;

    private boolean running = false;

    Thread ServoGoToPosition = null;

    Servo rotateServo = null;
    int target;

    final int UPPOSITION = 0, DOWNPOSITION = 0;

    public ServoGotToPositionSystem(Servo servo, Gamepad gm1){
        this.gm1 = gm1;
        this.rotateServo = servo;
    }
    
    public void start(){
        running=true;
        if(ServoGoToPosition == null || !ServoGoToPosition.isAlive()){
            ServoGoToPosition = new Thread(() ->{
                while(running){
                    if(gm1.y){
                        target = UPPOSITION;
                    }

                    if(gm1.a){
                        target = DOWNPOSITION;
                    }

                    rotateServo.setPosition(target);

                }
            });
        }

        ServoGoToPosition.start();
    }
    
    public void stop(){
        running = false;
    }
    
    public boolean getStatus(){
        return running;
    }
}
