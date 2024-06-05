package org.firstinspires.ftc.teamcode.Discover;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Discover.Threads.Blueprint;

@TeleOp
public class BasicOp extends LinearOpMode {
    
    /**
     * Examples of declaring
      */
    
    HardwareClass hardwareClass = HardwareClass.getInstance(hardwareMap);
    Blueprint blueprint = Blueprint.getInstance(hardwareMap,gamepad1);
    
    @Override
    public void runOpMode() {
        
        blueprint.start();
        
        waitForStart();
        
        while (opModeIsActive()) {
        
        }
    }}
