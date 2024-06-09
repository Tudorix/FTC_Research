package org.firstinspires.ftc.teamcode.CenterStage.Threads.Inputs;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Systems.ServoControlSystem;
import org.firstinspires.ftc.teamcode.Systems.SlidesSystem;
import org.firstinspires.ftc.teamcode.Systems.SweeperSystem;

public class InputThread implements Runnable {

    private static InputThread instance = null;

    private String threadName;
    private Thread thread;

    private boolean running;
    private final Gamepad gp1,gp2;

    private final Telemetry telemetry;
    
    private HardwareMap hardwareMap;
    
    //Gamepad 2
    SlidesSystem slidesSystem = null;
    SweeperSystem sweeperSystem = null;
    ServoControlSystem servoControlSystem = null;
    
    public boolean start(){
        try{
            if(this.thread == null || !this.thread.isAlive()){
                this.thread = new Thread(this);
                this.running = true;
                
                this.thread.start();
                return true;
            } else {
                return false;
            }
        } catch (IllegalThreadStateException e){
            this.telemetry.addData(this.threadName + " -> thread start error: ",e);
            telemetry.update();
            return false;
        }
    }
    
     public boolean stop(){
        try {
            if(this.thread != null){
                this.running = false;
                this.thread.join();
                return true;
            }else{
                this.telemetry.addData(this.threadName + " -> thread stop error", "is null");
                telemetry.update();
                return false;
            }
        } catch (InterruptedException e) {
            this.telemetry.addData(this.threadName + " -> thread stop error", e);
            telemetry.update();
            return false;
        }
    }


    @Override
    public void run() {
    
    }


    public InputThread(Gamepad gp1, Gamepad gp2, Telemetry telemetry, String threadName , HardwareMap hardwareMap){
        this.gp1 = gp1;
        this.gp2 = gp2;
        this.telemetry = telemetry;
        this.threadName = threadName;
        this.hardwareMap = hardwareMap;
    }


    public synchronized InputThread getInstance(Gamepad gp1, Gamepad gp2, Telemetry telemetry, String threadName , HardwareMap hardwareMap){
        if (instance == null){
            instance = new InputThread(gp1, gp2, telemetry, threadName , hardwareMap);
        }
        return instance;
    }
}