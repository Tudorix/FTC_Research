package org.firstinspires.ftc.teamcode.CenterStage.TeleOperated.TestingAndTuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Threads.SlidesPIDThread;
import org.firstinspires.ftc.teamcode.Vision.AprilTagSystem;
import org.firstinspires.ftc.teamcode.Vision.TSESystem;

//@TeleOp
//@Config
public class ListPosition_Testing extends OpMode {
    
    DcMotor LiftRight , LiftLeft = null;
    
    SlidesPIDThread slidesPIDThread = null;
    public static int target = 0 , kp = 0, ki = 0, kd = 0;
    
    @Override
    public void init(){
        LiftLeft = hardwareMap.get(DcMotor.class , "LS");
        LiftRight = hardwareMap.get(DcMotor.class,"RS");
        
        LiftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    
        LiftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LiftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesPIDThread = SlidesPIDThread.getInstance(hardwareMap);
        
    }
    
    @Override
    public void loop() {
        slidesPIDThread.setReference(target);
        slidesPIDThread.setCoefficients(kp,ki,kd);
    }
}

