package org.firstinspires.ftc.teamcode.Discover.PID;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
    private DcMotorEx motor;
    
    double repetitions=0;

    /*
    Explicit 'this' to state object type clearly
     */

    /**
     * @param motor - the motor to which the PID controller affects
     * @param telemetry - telemtry object for logging
     */
    public GeneralPID(DcMotorEx motor, String motorName, String threadName, Telemetry telemetry){
        this.motor = motor;
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
        this.motor = motor;
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
                this.current = motor.getCurrentPosition();
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
        
                current = motor.getCurrentPosition();
        
                error = reference - current;
                derivative = (error - lastError) / timer.seconds();
                integralSum = integralSum + (error * timer.seconds());
        
                double output = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
                
                motor.setPower(output);
        
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
            current = motor.getCurrentPosition();
            telemetry.addData(motorName + " pos", current);
            telemetry.update();
            repetitions++;
        }
    }


}
