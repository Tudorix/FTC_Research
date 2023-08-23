package org.firstinspires.ftc.teamcode.BRI;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import org.firstinspires.ftc.teamcode.threadopmode.*;

@TeleOp
public class TeleOp_BRI extends ThreadOpMode {
    
    Hardware_BRI robot = new Hardware_BRI();
    
    double integral = 0;
    double repetitions = 0;
    
    PIDCoefficients TestPID = new PIDCoefficients(0 ,0 ,0);
    
    ElapsedTime PIDTimer = new ElapsedTime();
    
    
    @Override
    public void mainInit(){
        robot.init(hardwareMap);
    
        //Below is a new thread
        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                PID_Controller(0 , robot.TurnOverArm, robot.TurnOverArm_Integral, robot.TurnOverArm_Repetitions);
            }
        }));
    }
    
    @Override
    public void mainLoop(){
    
    }
    
    
    /**
     *
     *  Delimitation
     *
     */
    
    
    
    void PID_Controller(double TargetPosition, DcMotor Motor, double integral, double repetitions){
        double Error = Motor.getCurrentPosition();
        double LastError = 0;
        
        while(Math.abs(Error) <= 8 && repetitions <= 40){ //   The actual magic :) | The numbers is just a test number and we can play with them until they are good
            Error = Motor.getCurrentPosition() - TargetPosition;
            double ChangeInError = LastError - Error;
            
            integral += ChangeInError * PIDTimer.time();
            double derivative = ChangeInError / PIDTimer.time();
            
            double P = TestPID.p * Error;
            double I = TestPID.i * integral;
            double D = TestPID.d * derivative;
    
            Motor.setPower(P + I + D);
            
            Error = LastError;
            repetitions++;
            PIDTimer.reset();
        }
        
    }
    
}