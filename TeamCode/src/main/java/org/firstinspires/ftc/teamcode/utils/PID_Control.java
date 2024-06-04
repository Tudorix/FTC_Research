package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name="", group = "")
//@Disabled
public class PID_Control extends LinearOpMode {
    
    //Declarations
    
    DcMotor TestMotor;
    /**
     *      The coefficients will be tuned
     */
    PIDCoefficients TestPID = new PIDCoefficients(0 ,0 ,0);
    
    ElapsedTime PIDTimer = new ElapsedTime();
    
    @Override
    public void runOpMode(){
        
        // after INIT
        
        TestMotor = hardwareMap.dcMotor.get("TEST");
        
        TestMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        TestMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TestMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       
        waitForStart();
        
        while (opModeIsActive()){
            //What happens after START
        }
        
    }
    
    void PID_Controller(double TargetPosition, double integral , double repetitions, DcMotor TestMotor, PIDCoefficients TestPID, ElapsedTime PIDTimer){
        double Error = TestMotor.getCurrentPosition();
        double LastError = 0;
        
        while(Math.abs(Error) <= 8 && repetitions <= 40){ //   The actual magic :) | The numbers is just a test number and we can play with them until they are good
            Error = TestMotor.getCurrentPosition() - TargetPosition;
            double ChangeInError = LastError - Error;
            
            integral += ChangeInError * PIDTimer.time();
            double derivative = ChangeInError / PIDTimer.time();
            
            double P = TestPID.p * Error;
            double I = TestPID.i * integral;
            double D = TestPID.d * derivative;
            
            TestMotor.setPower(P + I + D);
            
            Error = LastError;
            repetitions++;
            PIDTimer.reset();
        }
        
    }
}