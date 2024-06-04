package org.firstinspires.ftc.teamcode.Threads.Inputs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Systems.DistanceSystem;
import org.firstinspires.ftc.teamcode.Systems.ServoControlSystem;
import org.firstinspires.ftc.teamcode.Systems.SlidesSystem;
import org.firstinspires.ftc.teamcode.Systems.SweeperSystem;
import org.firstinspires.ftc.teamcode.Threads.GeneralPID;
import org.firstinspires.ftc.teamcode.Threads.SlidesPIDThread;

public class Gamepad2InputThread {
    
    Gamepad gm;
    private static Gamepad2InputThread single_instance = null;
    private boolean running = false;
    Thread gamepad2InputThread = null;
    
    int timeToWait_LONG = 500;
    
    //Systems
    //SlidesSystem slidesSystem = null;
    GeneralPID generalPID = null;
    SweeperSystem sweeperSystem = null;
    ServoControlSystem servoControlSystem = null;
    
    //SlidesPIDThread slidesPIDThread = null;
    
    HardwareMap hardwareMap = null;
    Telemetry telemetry = null;
    
    DcMotorEx TestMotor1 = null;
    DcMotorEx TestMotor2 = null;
    
    int var = 0;
    public Gamepad2InputThread(Gamepad gm , HardwareMap hardwareMap, Telemetry telemetry){
        this.gm = gm;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        //slidesSystem = SlidesSystem.getInstance(hardwareMap , telemetry);
        sweeperSystem = SweeperSystem.getInstance(hardwareMap);
        servoControlSystem = ServoControlSystem.getInstance(hardwareMap);
    }
    
    public void start(){
        running=true;
        stopSweepr();
        servoControlSystem.rotateArmTake();
        servoControlSystem.stopperOpen();
        
        //SLIDES
        TestMotor1 = hardwareMap.get(DcMotorEx.class , "LS");
        TestMotor2 = hardwareMap.get(DcMotorEx.class , "RS");
    
        TestMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        TestMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TestMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TestMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
        TestMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
        generalPID = new GeneralPID(TestMotor1 , TestMotor2 , "","",telemetry);
        generalPID.start();
        generalPID.setCoefficients(0.0008,0,0);
        if(gamepad2InputThread == null || !gamepad2InputThread.isAlive()){
            gamepad2InputThread = new Thread(() ->{
                while(running){
                    //Servos
                    if(gm.a && var == 0){ servoControlSystem.stopperClose(); var = 1;}
                    else if(gm.y){ importPixels(); var = 0;}
                    else if(gm.a && var == 1){ release1Pixel(); var = 2;}
                    else if(gm.a && var == 2) importPixels();
                    
                    if(gm.x) rectify();
                    
                    if(gm.b) pixelsPlace();
                    
                    if(gm.right_bumper) startSweepr();
                    else if(gm.left_bumper) stopSweepr();
                    
                    //Sweeper
                    //ksweeperSystem.setSweeperPower(gm.right_stick_x);
                    
                    //Slides
    
                    if(gm.dpad_up) generalPID.setReference(2000);
                    else if(gm.dpad_down) generalPID.setReference(-50);
                    else if(gm.dpad_right) generalPID.setReference(3000);
                    else if(gm.dpad_left) generalPID.setReference(1000);
                    
                    /*
                    if(gm.dpad_up) slidesSystem.slidesUp();
                    else if(gm.dpad_down) slidesSystem.slidesDown();
                    else if(gm.dpad_right) slidesSystem.holdSLides();
                    else slidesSystem.stopPowerSlides();
                    */
                }
            });
        }
        gamepad2InputThread.start();
    }
    
    private void startSweepr(){
        servoControlSystem.sweepDown();
        try{
            Thread.sleep(timeToWait_LONG);
        }catch (Exception e){}
        sweeperSystem.startSweeper();
    }
    
    private void stopSweepr(){
        sweeperSystem.stopSweeper();
        try{
            Thread.sleep(timeToWait_LONG);
        }catch (Exception e){}
        servoControlSystem.sweepUp();
    }
    
    private void release1Pixel(){
        servoControlSystem.stopperOut();
        try{
            Thread.sleep(100);
        }catch (Exception e){}
        servoControlSystem.stopperClose();
    }
    
    private void pixelsPlace(){
        servoControlSystem.adjust();
        var = 1;
        try{
            Thread.sleep(200);
        }catch (Exception e){}
        servoControlSystem.stopperClose();
        try{
            Thread.sleep(200);
        }catch (Exception e){}
        servoControlSystem.rotateArmPlace();
    }
    
    private void rectify(){
        servoControlSystem.adjust();
        servoControlSystem.stopperClose();
        try{
            Thread.sleep(200);
        }catch (Exception e){}
        servoControlSystem.rotateArmTake();
        servoControlSystem.stopperOpen();
    }
    
    private void importPixels(){
        servoControlSystem.stopperOpen();
        try{
            Thread.sleep(400);
        }catch (Exception e){}
        servoControlSystem.goDown();
    }
    
    public void stop(){
        running = false;
    }
    
    public boolean getStatus(){
        return running;
    }
    
    public static synchronized Gamepad2InputThread getInstance(HardwareMap hardwareMap, Gamepad gm , Telemetry telemetry){
        if(single_instance == null){
            single_instance = new Gamepad2InputThread(gm , hardwareMap , telemetry);
        }
        return single_instance;
    }
}
