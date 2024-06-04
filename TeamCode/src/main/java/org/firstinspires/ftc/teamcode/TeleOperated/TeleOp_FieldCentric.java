package org.firstinspires.ftc.teamcode.TeleOperated;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Systems.ServoControlSystem;
import org.firstinspires.ftc.teamcode.Systems.SlidesSystem;
import org.firstinspires.ftc.teamcode.Systems.SweeperSystem;

@TeleOp
@Config
public class TeleOp_FieldCentric extends LinearOpMode {
    Robot robot = null;
    ServoControlSystem servoControlSystem = null;
    SlidesSystem slidesSystem = null;
    SweeperSystem sweeperSystem = null;
    
    @Override
    public void runOpMode() throws InterruptedException {
        robot = Robot.getInstance(hardwareMap);
        DcMotorEx frontLeftMotor = robot.FL;
        DcMotorEx backLeftMotor = robot.BL;
        DcMotorEx frontRightMotor = robot.FR;
        DcMotorEx backRightMotor = robot.BR;
        
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
    
        slidesSystem = SlidesSystem.getInstance(hardwareMap);
        servoControlSystem = ServoControlSystem.getInstance(hardwareMap);
        sweeperSystem = SweeperSystem.getInstance(hardwareMap);
        
        waitForStart();
        
        if (isStopRequested()) return;
        
        while (opModeIsActive()) {
            
            /**
             * DRIVETRAIN | GM1
             */
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            
            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.y) {
                imu.resetYaw();
            }
            
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            
            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            
            rotX = rotX * 1.1;  // Counteract imperfect strafing
            
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;
            
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            
            /**
             *  INTAKE & OUTTAKE | GM2
             */
            
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
}
