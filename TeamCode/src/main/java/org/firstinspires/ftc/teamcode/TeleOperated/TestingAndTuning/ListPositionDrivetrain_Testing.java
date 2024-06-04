package org.firstinspires.ftc.teamcode.TeleOperated.TestingAndTuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//@TeleOp
//@Config
public class ListPositionDrivetrain_Testing extends LinearOpMode {
    
    DcMotor FR , FL , BR , BL;
    
    @Override
    public void runOpMode() throws InterruptedException {
        BR = hardwareMap.get(DcMotor.class , "BR");
        BL = hardwareMap.get(DcMotor.class , "FR");
        FR = hardwareMap.get(DcMotor.class , "BL");
        FL = hardwareMap.get(DcMotor.class , "FL");
        
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
    
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
        waitForStart();
    
        if (isStopRequested()) return;
    
        while (opModeIsActive()) {
            telemetry.addLine("FL Position: " + FL.getCurrentPosition());
            telemetry.addLine("FR Position: " + FR.getCurrentPosition());
            telemetry.addLine("BL Position: " + BL.getCurrentPosition());
            telemetry.addLine("BR Position: " + BR.getCurrentPosition());
            telemetry.update();
        }
    }
}

