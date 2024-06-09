package org.firstinspires.ftc.teamcode.CenterStage.TeleOperated;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Auxiliary.PIDConstants;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Systems.ServoControlSystem;
import org.firstinspires.ftc.teamcode.Systems.SlidesSystem;
import org.firstinspires.ftc.teamcode.Systems.SweeperSystem;
import org.firstinspires.ftc.teamcode.Threads.Drive.DriveTrainThread;
import org.firstinspires.ftc.teamcode.Threads.Drive.StataicHeading_Drive;
import org.firstinspires.ftc.teamcode.Threads.Inputs.InputThread;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;


//@TeleOp
//@Config
public class StaticHeading_TeleOp extends LinearOpMode {
    double integralSum = 0;
    public static double Kp = 1 ;
    public static double Ki = 0;
    public static double Kd = 0.05;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    
    StataicHeading_Drive stataicHeading_drive = null;

    @Override
    public void runOpMode() throws InterruptedException {
        
        IMU imu;
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        
        //Input
        stataicHeading_drive = StataicHeading_Drive.getInstance(hardwareMap , gamepad1);
        stataicHeading_drive.start();
        
        double refrenceAngle = Math.toRadians(270);
        waitForStart();
        
        while(opModeIsActive()) {
            telemetry.addData("Target IMU Angle", refrenceAngle);
            telemetry.addData("Current IMU Angle", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            double power = PIDControl(refrenceAngle, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            stataicHeading_drive.orientation = power;
        }
    }

    public double PIDControl(double refrence, double state) {
        double error = angleWrap(refrence - state);
        telemetry.addData("Error: ", error);
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / (timer.seconds());
        lastError = error;
        timer.reset();
        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }
    
    public double angleWrap(double radians){
        while(radians > Math.PI){
            radians -= 2 * Math.PI;
        }
        while(radians < -Math.PI){
            radians += 2 * Math.PI;
        }
        return radians;
    }
}