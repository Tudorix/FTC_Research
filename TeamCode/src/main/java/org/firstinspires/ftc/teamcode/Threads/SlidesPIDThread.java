package org.firstinspires.ftc.teamcode.Threads;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Systems.DistanceSystem;

public class SlidesPIDThread {
    
    Gamepad gm;
    private static SlidesPIDThread single_instance = null;
    private boolean running = false;
    Thread slidesPIDThread = null;
    
    private double Kp, Ki, Kd;
    private double reference, current = 0;
    private double integralSum, derivative;
    private double error, lastError;
    
    double repetitions=0;
    
    DcMotor Slide_YES_Encoder = null, Slide_NO_Encoder = null;
    
    
    public SlidesPIDThread(Robot instance){
        this.gm = gm;
        Slide_YES_Encoder = instance.LS;
        Slide_NO_Encoder = instance.RS;
    }
    
    public void start(){
        running=true;
        Slide_YES_Encoder.setDirection(DcMotorSimple.Direction.REVERSE);
        if(slidesPIDThread == null || !slidesPIDThread.isAlive()){
            slidesPIDThread = new Thread(() ->{
                while(running){
                    ElapsedTime timer = new ElapsedTime();
                    while (Math.abs(error) > 5 || repetitions < 40) {
        
                        current = Slide_YES_Encoder.getCurrentPosition();
        
                        error = reference - current;
                        derivative = (error - lastError) / timer.seconds();
                        integralSum = integralSum + (error * timer.seconds());
        
                        double output = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
    
                        Slide_YES_Encoder.setPower(output);
                        Slide_NO_Encoder.setPower(output);
        
                        lastError = error;
                        timer.reset();
                    }
                    current = Slide_YES_Encoder.getCurrentPosition();
                    repetitions++;
                }
            });
        }
        slidesPIDThread.start();
    }
    
    public void setCoefficients(double p, double i, double d){
        this.Kp = p;
        this.Ki = i;
        this.Kd = d;
    }
    
    public void setReference(double reference){
        this.reference = reference;
    }
    
    public void stop(){
        running = false;
    }
    
    public boolean getStatus(){
        return running;
    }
    
    public static synchronized SlidesPIDThread getInstance(HardwareMap hMap){
        if(single_instance == null){
            single_instance = new SlidesPIDThread(Robot.getInstance(hMap));
        }
        return single_instance;
    }
}
