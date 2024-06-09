package org.firstinspires.ftc.teamcode.Discover.Autonomous.Op;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Discover.Autonomous.Systems.EncoderChassis;
import org.firstinspires.ftc.teamcode.Discover.HardwareClass;

@Autonomous
public class Autonomous_TestOp extends LinearOpMode {
    
    HardwareClass hardwareClass = null;
    @Override
    public void runOpMode() throws InterruptedException {
        
        hardwareClass = HardwareClass.getInstance(hardwareMap);
        EncoderChassis encoderChassis = new EncoderChassis(hardwareClass);
        
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            encoderChassis.Movement(0,0,0,0,0);
        }
    }
}