package org.firstinspires.ftc.teamcode.CenterStage.TeleOperated.TestingAndTuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Threads.GeneralPID;

@TeleOp(name="Slides Pid Tuning", group = "")
@Config
public class PID_Control extends LinearOpMode {
    
    DcMotorEx TestMotor1 = null;
    DcMotorEx TestMotor2 = null;
    public static double kp = 0, ki = 0, kd = 0;
    public static double target = 100;
    GeneralPID generalPID = null;
    
    @Override
    public void runOpMode(){
        
        // after INIT
        TestMotor1 = hardwareMap.get(DcMotorEx.class , "LS");
        TestMotor2 = hardwareMap.get(DcMotorEx.class , "RS");
        
        TestMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        TestMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TestMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TestMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
        TestMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        waitForStart();
 
        generalPID = new GeneralPID(TestMotor1 , TestMotor2 , "","",telemetry);
        generalPID.start();
        
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive()){
            //What happens after START
            generalPID.setReference(target);
            generalPID.setCoefficients(kp,ki,kd);
            
        }
        
        generalPID.stop();
        
    }
}