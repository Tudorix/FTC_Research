package org.firstinspires.ftc.teamcode.Systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Threads.GeneralPID;
import org.firstinspires.ftc.teamcode.Robot;

@Config
public class SlidesSystem {
    private static SlidesSystem single_instance = null;
    private boolean running = false;
    Thread SlidesSystem = null;
    ServoControlSystem servoControlSystem = null;

    DcMotorEx LeftSlide , RightSlide;
    int MAX_POSITION = 600 , DOWN_POSITION = 20;
    Gamepad gm;
    
    HardwareMap hmap = null;
    Telemetry telemetry = null;
    
    double  MaxPower = 0.7 , Equal = 0.8;
    GeneralPID generalPIDLeft = null;
    GeneralPID generalPIDRight = null;
    
    public SlidesSystem(Robot instance , Telemetry telemetry){
        this.LeftSlide = instance.LS;
        this.RightSlide = instance.RS;
        this.hmap = instance.hardwareMap;
        this.telemetry = telemetry;
        resetSlidesEncoders();
    }
    
    /**
     * PID loop
     */
    
    public void startSlides(){
        //The Slides Thread
        generalPIDLeft = new GeneralPID(LeftSlide, RightSlide , "Slides" , "SlidesThread" , telemetry);
    
        generalPIDLeft.start();
    
        generalPIDLeft.setCoefficients(Robot.getInstance(hmap).kP, Robot.getInstance(hmap).kI , Robot.getInstance(hmap).kD);
    }
    
    public void stopSlides(){
        generalPIDRight.stop();
    }
    public void slideSetTarget(int target){
        generalPIDLeft.setReference(target);
    }
    
    
    /**
     * Manual control
     */
    public void slidesUp(){
        LeftSlide.setPower(MaxPower);
        RightSlide.setPower(MaxPower);
    }
    
    public void slidesDown(){
        LeftSlide.setPower(-MaxPower * 0.6);
        RightSlide.setPower(-MaxPower * 0.6);
    }
    
    /**
     * Give fixed power to slides
     */
    
    public void holdSLides(){
        LeftSlide.setPower(Equal);
        RightSlide.setPower(Equal);
    }
    
    public void stopPowerSlides(){
        LeftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftSlide.setPower(0);
        RightSlide.setPower(0);
    }
    
    /**
     * Slides go to position
     */
    
    public void slidesToPosition_Custom(int Position, double Power){
        //Set target position
        LeftSlide.setTargetPosition(Position);
        RightSlide.setTargetPosition(Position);
    
        //Set motors to run to position
        LeftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        int sign;
        if(LeftSlide.getCurrentPosition() < Position && RightSlide.getCurrentPosition() < Position){
            sign = 1;
        }else{
            sign = -1;
        }
    
        //Set power to all motors
        LeftSlide.setPower(Power * sign * MaxPower);
        RightSlide.setPower(Power * sign * MaxPower);
    
        while(LeftSlide.isBusy() && RightSlide.isBusy()){
            //Pause loop for the slides to get to position
        }
    
        //Stop motors
        LeftSlide.setPower(0);
        RightSlide.setPower(0);
    
        LeftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    public void resetSlidesEncoders(){
        LeftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
        LeftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    public static synchronized SlidesSystem getInstance(HardwareMap hMap , Telemetry telemetry){
        if(single_instance == null){
            single_instance = new SlidesSystem(Robot.getInstance(hMap) , telemetry);
        }
        return single_instance;
    }
}
