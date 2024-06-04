package org.firstinspires.ftc.teamcode.TeleOperated.TestingAndTuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Vision.AprilTagSystem;
import org.firstinspires.ftc.teamcode.Vision.TSESystem;

@Config
//@TeleOp
public class Cam_Testing extends OpMode {
    
    TSESystem tseSystem = null;
    AprilTagSystem aprilTagSystem = null;
    
    @Override
    public void init(){
        tseSystem = TSESystem.getInstance(hardwareMap);
        tseSystem.start("red");
    }
    
    @Override
    public void loop(){
        telemetry.addLine(tseSystem.SeeCase() + " ");
    }
    
}

