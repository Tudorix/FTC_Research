package org.firstinspires.ftc.teamcode.Threads;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.util.DashboardUtil;
import org.opencv.core.Mat;

import java.util.prefs.PreferencesFactory;

public class GeneralPID implements Runnable{

    private volatile boolean running = false;
    private String threadName = null;
    private String motorName = null;
    private Telemetry telemetry;
    private Thread thread = null;

    private double Kp, Ki, Kd;
    private double reference, current = 0;
    private double integralSum, derivative;
    private double error, lastError;
    private DcMotorEx motor1 , motor2;
    
    double repetitions=0;

    /*
    Explicit 'this' to state object type clearly
     */

    /**
     * @param motor1 - the motor to which the PID controller affects
     * @param telemetry - telemtry object for logging
     */
    public GeneralPID(DcMotorEx motor1, DcMotorEx motor2, String motorName, String threadName, Telemetry telemetry){
        this.motor1 = motor1;
        this.motor2 = motor2;
        this.motorName = motorName;
        this.threadName = threadName;
        this.telemetry = telemetry;
    }

    public GeneralPID(DcMotorEx motor1,
                      DcMotorEx motor2,
                      double kp,
                      double ki,
                      double kd,
                      double reference,
                      String motorName,
                      String threadName,
                      Telemetry telemetry){
        this.motor1 = motor1;
        this.motor2 = motor2;
        this.Kp = kp;
        this.Ki = ki;
        this.Kd = kd;
        this.motorName = motorName;
        this.threadName = threadName;
        this.telemetry = telemetry;
    }
    public boolean start(){
        try {
            if (this.thread == null || !this.thread.isAlive()) {
                telemetry.addLine("Trying to start");
                telemetry.update();
                this.thread = new Thread(this);
                this.running = true;
                this.current = motor1.getCurrentPosition();
                this.repetitions=0;
                this.thread.start();

                return true;
            } else{
                return false;
            }
        }catch (IllegalThreadStateException e){
            this.telemetry.addData(this.threadName + " -> thread start error: ",e);
            telemetry.update();
            return false;
        }
    }

    public boolean stop(){
        try {
            if (this.thread != null) {
                this.running = false;
                this.thread.join();
                return true;
            } else {
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

    /**
     * @param p - kp
     * @param i - ki
     * @param d - kd
     */
    public void setCoefficients(double p, double i, double d){
        this.Kp = p;
        this.Ki = i;
        this.Kd = d;
    }

    /**
     * @param reference - desired position
     */
    public void setReference(double reference){
       this.reference = reference;
    }
    
    FtcDashboard ftdb = FtcDashboard.getInstance();
    
    @Override
    public void run() {
        // short amount of time resets in every loop so every point of the
        while(running) {
            ElapsedTime timer = new ElapsedTime();
            while (Math.abs(error) > 5 || repetitions < 40) {
        
                current = motor1.getCurrentPosition();
        
                error = reference - current;
                derivative = (error - lastError) / timer.seconds();
                integralSum = integralSum + (error * timer.seconds());
        
                double output = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
                
                motor1.setPower(output);
                motor2.setPower(output);
        
                telemetry.addData(motorName + " target", reference);
                telemetry.addData(motorName + " pos", current);
                
                TelemetryPacket tp = new TelemetryPacket();
                tp.put("pos", current);
                tp.put("ref", reference);
                tp.put("error", error);
                ftdb.sendTelemetryPacket(
                        tp
                );
        
                telemetry.update();
                
                lastError = error;
                timer.reset();
            }
            current = motor1.getCurrentPosition();
            telemetry.addData(motorName + " pos", current);
            telemetry.update();
            repetitions++;
        }
    }


}
