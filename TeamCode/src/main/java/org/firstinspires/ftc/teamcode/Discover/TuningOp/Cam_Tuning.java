package org.firstinspires.ftc.teamcode.Discover.TuningOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Discover.Vision.AprilTagSystem;
import org.firstinspires.ftc.teamcode.Discover.Vision.TSESystem;

@Config
@TeleOp
public class Cam_Tuning extends OpMode {
    
    TSESystem tseSystem = null;
    AprilTagSystem aprilTagSystem = null;
    
    @Override
    public void init(){
    
        /**
         * ONLY ONE AT THE TIME
         */
    
        //tseSystem = TSESystem.getInstance(hardwareMap);
        //tseSystem.start("red"); // or "blue"
        
        aprilTagSystem = AprilTagSystem.getInstance(hardwareMap);
        aprilTagSystem.start();
    }
    
    @Override
    public void loop(){
        telemetry.addLine(aprilTagSystem.SeeTag() + " ");
        //telemetry.addLine(tseSystem.SeeCase() + " ");
    }
    
}

