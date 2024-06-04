package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot;

@Config
public class IntakeSystem {

    public static double intakePower = 0.8;
    public static double outtakePower = -0.8;
    public static double zonePutPower = -0.4;
    private static IntakeSystem intakeSystem = null;

    private DcMotorEx intakeMotor = null;
    private IntakeSystem(Robot robot){
        this.intakeMotor = robot.intakeMotor;
    }

    public void start(){
        this.intakeMotor.setPower(intakePower);
    }
;
    public void throwPixel(){
        this.intakeMotor.setPower(outtakePower);
    }
    
    public void setPowerIntake(double Power){
        this.intakeMotor.setPower(Power);
    }

    public void placePixel(){
        this.intakeMotor.setPower(zonePutPower);
    }

    public void stop(){
        this.intakeMotor.setPower(0);
    }

    public static synchronized IntakeSystem getInstance(Robot robot){
        if(intakeSystem == null)
            intakeSystem = new IntakeSystem(robot);
        return  intakeSystem;
    }
}
