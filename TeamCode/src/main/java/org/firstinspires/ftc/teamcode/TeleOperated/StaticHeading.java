package org.firstinspires.ftc.teamcode.TeleOperated;

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
import org.firstinspires.ftc.teamcode.Threads.Inputs.Gamepad2InputThread;
import org.firstinspires.ftc.teamcode.Threads.Inputs.InputThread;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;


//@TeleOp(name = "Balkan4Life")
//@Config
public class StaticHeading extends LinearOpMode {
    double integralSum = 0;
    public static double Kp = 1;
    public static double Ki = 0;
    public static double Kd = 0.05;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    
    DriveTrainThread driveTrainSystem = null;
    Gamepad2InputThread gamepad2InputThread = null;

    @Override
    public void runOpMode() throws InterruptedException {
        
        IMU imu;
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
        imu.initialize(parameters);
        
        //Drive
        driveTrainSystem = DriveTrainThread.getInstance(hardwareMap , gamepad1);
        driveTrainSystem.start();
        
        //Gamepad 2
        gamepad2InputThread = Gamepad2InputThread.getInstance(hardwareMap , gamepad2 ,telemetry);
        gamepad2InputThread.start();
        
        double refrenceAngle = Math.toRadians(270);
        waitForStart();
        
        while(opModeIsActive()) {
            telemetry.addData("Target IMU Angle", refrenceAngle);
            telemetry.addData("Current IMU Angle", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            double power = PIDControl(refrenceAngle, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            driveTrainSystem.orientation = power;
        }
    }

    public double PIDControl(double refrence, double state) {
        double error = angleWrap(refrence - state);
        telemetry.addData("Error: ", error);
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / (timer.seconds());
        lastError = error;
        if(gamepad1.a){
            lastError = 0;
        }
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