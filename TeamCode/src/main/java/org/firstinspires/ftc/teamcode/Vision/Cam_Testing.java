package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Config
public class Cam_Testing extends OpMode {
    
    TSESystem tseSystem = null;
    AprilTagSystem aprilTagSystem = null;
    
    @Override
    public void init(){
        tseSystem = TSESystem.getInstance(hardwareMap);
        tseSystem.start("blue");
    }
    @Override
    public void loop(){
        telemetry.addLine(tseSystem.SeeCase() + " ");
    }
    
}

