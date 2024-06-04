package org.firstinspires.ftc.teamcode.Threads.Inputs;

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
                
                //Slides
                slidesSystem = SlidesSystem.getInstance(hardwareMap, telemetry);
                
                //Sweeper
                sweeperSystem = SweeperSystem.getInstance(hardwareMap);
                
    
    
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
        while(running){
            if(gp2.right_bumper){
                servoControlSystem.sweepDown();
            }else if(gp2.left_bumper){
                servoControlSystem.sweepUp();// Calculate max height position
            }else if(gp2.dpad_down){
                slidesSystem.slidesDown();
            }else if(gp2.dpad_right){
                slidesSystem.holdSLides();
            }else if(gp2.dpad_up){
                slidesSystem.slidesUp();
            }else{
                slidesSystem.stopPowerSlides();
            }
    
            if(gp2.a){
                servoControlSystem.stopperClose();
            }else if (gp2.b){
                servoControlSystem.rotateArmPlace();
            }else if(gp2.x){
                servoControlSystem.rotateArmTake();
            }else if (gp2.y){
                servoControlSystem.stopperOpen();
            }else if (gp2.right_bumper && gp2.left_bumper){
                servoControlSystem.launchPLane();
            }
            
            /*if(gp2.dpad_down){
                slidesSystem.slideSetTarget(0);
            }else if(gp2.dpad_right){
                slidesSystem.slideSetTarget(3000);
            }else if(gp2.dpad_left){
                slidesSystem.slideSetTarget(1000);
            }else if(gp2.dpad_up){
                slidesSystem.slideSetTarget(2000);
            }*/
    
           /* if(gp2.dpad_down){
                servoControlSystem.stopperOpen();
                try{
                    Thread.sleep(1000);
                }catch (InterruptedException e){};
                servoControlSystem.rotateArmTake();
                try{
                    Thread.sleep(500);
                }catch (InterruptedException e){};
                slidesSystem.slideSetTarget(0);
                servoControlSystem.sweepDown();
            }else if(gp2.dpad_up){
                servoControlSystem.stopperClose();
                slidesSystem.slideSetTarget(700);// Calculate max height position
                try{
                    Thread.sleep(500);
                }catch (InterruptedException e){};
                servoControlSystem.rotateArmPlace();
                servoControlSystem.sweepUp();
            }else if(gp2.dpad_right){
                servoControlSystem.stopperClose();
                slidesSystem.slideSetTarget(1000);
                try{
                    Thread.sleep(500);
                }catch (InterruptedException e){};
                servoControlSystem.rotateArmPlace();
                servoControlSystem.sweepUp();
            }else if(gp2.dpad_left){
                servoControlSystem.stopperClose();
                slidesSystem.slideSetTarget(1300);// Calculate max height position
                try{
                    Thread.sleep(500);
                }catch (InterruptedException e){};
                servoControlSystem.rotateArmPlace();
                servoControlSystem.sweepUp();
            }
            */
            
            
            sweeperSystem.setSweeperPower(gp2.right_stick_x);
            //...
        }
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