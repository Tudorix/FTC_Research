package org.firstinspires.ftc.teamcode.Systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;

@Config
public class SlidesSystem {
    private static SlidesSystem single_instance = null;
    private boolean running = false;
    Thread SlidesSystem = null;
    ServoControlSystem servoControlSystem = null;

    DcMotor LeftSlide , RightSlide;
    int MAX_POSITION = 600 , DOWN_POSITION = 20;
    Gamepad gm;
    
    HardwareMap hmap = null;
    
    double  MaxPower , Equal;
    public SlidesSystem(Robot instance){
        this.LeftSlide = instance.LS;
        this.RightSlide = instance.RS;
        LeftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        this.gm = gm;
        this.MaxPower = instance.MAX_POWER_SLIDES;
        this.Equal = instance.SLIDES_EQUILIBRIUM;
        this.hmap = instance.hardwareMap;
    }
    
    /**
     * Give custom power to slides
     */
    
    public void slidesUp(){
        LeftSlide.setPower(MaxPower);
        RightSlide.setPower(MaxPower);
    }
    
    public void slidesDown(){
        LeftSlide.setPower(-MaxPower);
        RightSlide.setPower(-MaxPower);
    }
    
    /**
     * Give fixed power to slides
     */
    
    public void holdSLides(){
        LeftSlide.setPower(Equal);
        RightSlide.setPower(Equal);
    }
    
    public void stopSlides(){
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
    
    public static synchronized SlidesSystem getInstance(HardwareMap hMap){
        if(single_instance == null){
            single_instance = new SlidesSystem(Robot.getInstance(hMap));
        }
        return single_instance;
    }
}
