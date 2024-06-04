package org.firstinspires.ftc.teamcode.TestingAndTuning;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot;

public class DriveTrain_FC {

    DcMotor turnOverMotor = null;
    Gamepad gm1;

    double  maxPower;
    private static DriveTrain_FC single_instance = null;
    private boolean running = false;
    private DcMotor FR , FL , BR , BL;
    private double BLPower , BRPower , FRPower , FLPower;
    private double power , turn , max , x , y;
    private double sin , cos , theta;
    Thread DriveTrain_FC = null;

    IMU imu = null;
    IMU imu2 = null;

    public DriveTrain_FC(Robot instance, Gamepad gm1){
        this.gm1 = gm1;
        this.maxPower = instance.MAX_POWER_DRIVETRAIN;
        this.FR = instance.FR;
        this.FL = instance.FL;
        this.BR = instance.BR;
        this.BL = instance.BL;
        this.imu = instance.imu;
    }
    
    public void start(){
        running=true;
        if(DriveTrain_FC == null || !DriveTrain_FC.isAlive()){
            DriveTrain_FC = new Thread(() ->{
                while(running){
                    double y = -gm1.left_stick_y; // Remember, Y stick value is reversed
                    double x = gm1.left_stick_x;
                    double rx = gm1.right_stick_x;

                    // This button choice was made so that it is hard to hit on accident,
                    // it can be freely changed based on preference.
                    // The equivalent button is start on Xbox-style controllers.
                    if (gm1.y) {
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

                    FL.setPower(frontLeftPower);
                    BL.setPower(backLeftPower);
                    FR.setPower(frontRightPower);
                    BR.setPower(backRightPower);
                }
            });
        }
        DriveTrain_FC.start();
    }
    
    public void stop(){
        running = false;
    }
    
    public boolean getStatus(){
        return running;
    }
    
    public static synchronized DriveTrain_FC getInstance(HardwareMap hMap, Gamepad gm){
        if(single_instance == null){
            single_instance = new DriveTrain_FC(Robot.getInstance(hMap), gm);
        }
        return single_instance;
    }
}
