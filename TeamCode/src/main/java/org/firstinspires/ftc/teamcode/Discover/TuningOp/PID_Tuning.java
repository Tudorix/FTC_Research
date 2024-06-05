package org.firstinspires.ftc.teamcode.Discover.TuningOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Discover.PID.GeneralPID;

@TeleOp(name="Pid Tuning", group = "")
@Config
public class PID_Tuning extends LinearOpMode {
    
    DcMotorEx Motor = null;
    public static double kp = 0, ki = 0, kd = 0;
    public static double target = 100;
    GeneralPID generalPID = null;
    
    @Override
    public void runOpMode(){
        
        Motor = hardwareMap.get(DcMotorEx.class , "---MOTOR_NAME---");
        waitForStart();
 
        generalPID = new GeneralPID(Motor ,"","",telemetry);
        generalPID.start();
        
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive()){
            generalPID.setReference(target);
            generalPID.setCoefficients(kp,ki,kd);
            
        }
        generalPID.stop();
    }
}