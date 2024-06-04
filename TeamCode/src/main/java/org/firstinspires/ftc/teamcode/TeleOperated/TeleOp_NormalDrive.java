package org.firstinspires.ftc.teamcode.TeleOperated;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Threads.DriveTrainThread;
import org.firstinspires.ftc.teamcode.Systems.ServoControlSystem;
import org.firstinspires.ftc.teamcode.Systems.SlidesSystem;
import org.firstinspires.ftc.teamcode.Systems.SweeperSystem;

@TeleOp
@Config
public class TeleOp_NormalDrive extends OpMode {

    DriveTrainThread driveTrainSystem = null;
    ServoControlSystem servoControlSystem = null;
    SlidesSystem slidesSystem = null;
    SweeperSystem sweeperSystem = null;
    
    @Override
    public void init(){
        driveTrainSystem = DriveTrainThread.getInstance(hardwareMap , gamepad1);
        driveTrainSystem.start();
        
        //Gamepad 2
        slidesSystem = SlidesSystem.getInstance(hardwareMap);
        servoControlSystem = ServoControlSystem.getInstance(hardwareMap);
        sweeperSystem = SweeperSystem.getInstance(hardwareMap);
    }

    @Override
    public void loop(){
        //Gamepad 2
        if(gamepad2.dpad_left && gamepad2.left_bumper) servoControlSystem.Launch_PLane();
        
        if(gamepad2.x) servoControlSystem.Arm_Take();
        if(gamepad2.b) servoControlSystem.Arm_Place();
        
        if(gamepad2.a) servoControlSystem.Stopper_Close();
        if(gamepad2.y) servoControlSystem.Stopper_Open();
        
        sweeperSystem.setSweeperPower(gamepad2.right_stick_x);
        
        if(gamepad2.right_trigger > 0.5) slidesSystem.slidesUp();
        else if(gamepad2.left_trigger > 0.5) slidesSystem.slidesDown();
        else if(gamepad2.right_bumper) slidesSystem.holdSLides();
        else slidesSystem.stopSlides();
    }
}
