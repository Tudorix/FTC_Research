package org.firstinspires.ftc.teamcode.PowerPlay.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.hardware.Gamepad;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.ColorSensor;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.sql.Time;
import java.text.RuleBasedCollator;

@Config
@TeleOp(name="TeleOp_PowerPlay", group = "")
//@Disabled
public class TeleOp_PowerPlay extends LinearOpMode {

    //HardwareMecanum robot = new HardwareMecanum();
    public DcMotor FL = null;
    public DcMotor FR = null;
    public DcMotor BL = null;
    public DcMotor BR = null;

    public DcMotor String = null;
    public DcMotor String2 = null;

    public Servo Clamp = null;
    public CRServo Rul = null;

    //Variables
    double x = 0, y = 0;
    double  cos = 0 , sin = 0 ,  theta = 0;
    double max = 0;
    double  ClampPosition = 0 ;
    double  FLPower = 0 , FRPower = 0, BLPower = 0 , BRPower = 0;
    double turn = 0 , power = 0;
    double Trig_Forc = 0;
    double rullerPos = 0;
    long Timer = 0;
    long multiplier = 0;
    int Mode = 1;

    double Target_Height =  0;
    double StringSpeed = 1;

    double Count = 0;

    //Froebar

    FtcDashboard dashboard;
    public DcMotor Turn_Over = null;

    private PIDController controller;

    public static double p = 0, i = 0 , d = 0;
    public static double f = 0;

    public static double target_position = 1;

    private final double ticks_in_degree = 1120 / 360.0;

    private double lastError = 0;

    double Move_Speed = 0.7;

    ColorSensor colors;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode(){

        //robot.init(hardwareMap);
        /**
         * Motors
         */
        //Get hardware for motors

        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");

       colors = hardwareMap.get(ColorSensor.class , "CC");

        String = hardwareMap.dcMotor.get("LM");
        String2 = hardwareMap.dcMotor.get("LM2");

        Clamp= hardwareMap.servo.get("CL");
        Rul=hardwareMap.crservo.get("RL");

        //Initializing Forebar Components
        Turn_Over = hardwareMap.dcMotor.get("OH");
        dashboard = FtcDashboard.getInstance();

        //Set direction to all motors

        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        String2.setDirection(DcMotor.Direction.REVERSE);

        Turn_Over.setDirection(DcMotor.Direction.REVERSE);

        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        String.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        String2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Setting motors to run without encoders
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        String.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        String2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Turn_Over.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Turn_Over.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Turn_Over.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()){

            //Gamepad 1

            //Changing mode
            if(gamepad1.y){
                Mode *= -1;
            }

            if(Mode > 0){
                if(gamepad1.dpad_right){
                    x = ( Move_Speed + Trig_Forc);
                    y = 0;
                }else  if(gamepad1.dpad_left){
                    x = -(Move_Speed + Trig_Forc);
                    y = 0;
                }else  if(gamepad1.dpad_up){
                    x = 0;
                    y = -(Move_Speed + Trig_Forc);
                }else  if(gamepad1.dpad_down){
                    x = 0;
                    y = (Move_Speed +  Trig_Forc);
                }else{
                    x = 0;
                    y = 0;
                }

                Trig_Forc = gamepad1.left_trigger;
                telemetry.addLine("Driving mode: Dpad");
            }else{
                x = gamepad1.left_stick_x * Move_Speed;
                y = gamepad1.left_stick_y * Move_Speed;
                telemetry.addLine("Driving mode: Joystick");
            }

            //Mecanum Drive
            turn = gamepad1.right_stick_x * 0.6;

            //Multiplier = (-x + y)/5;
            theta = Math.atan2(y , x);
            power = Math.hypot(x, y);

            sin = Math.sin(theta - Math.PI/4);
            cos = Math.cos(theta - Math.PI/4);
            max = Math.max(Math.abs(sin) , Math.abs(cos));

            FLPower = power * cos/max + turn;
            FRPower = power * sin/max - turn;
            BLPower = power * sin/max + turn;
            BRPower = power * cos/max - turn;

            if((power + Math.abs(turn)) > 1){
                FLPower /= power + turn;
                FRPower /= power + turn;
                BLPower /= power + turn;
                BRPower /= power + turn;
            }

            FL.setPower(FLPower);
            FR.setPower(FRPower);
            BL.setPower(BLPower);
            BR.setPower(BRPower);

            //Reset encoder values
            if(gamepad1.x){
                FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                String.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                String2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                String.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//Change if you change how they function
                String2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            //Gamepad 2

            //String Motors

            rullerPos--;
            if(gamepad2.left_trigger > 0 && rullerPos < 0){
                rullerPos = 20;
                target_position *= -1;
            }

            if(target_position == -1){ // Change Modes

                String.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                String2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if(gamepad2.y && (!String.isBusy() || !String2.isBusy())){
                Target_Height = -1700;//Change

                String.setTargetPosition((int) Target_Height);
                String2.setTargetPosition((int) Target_Height);

                String.setMode(DcMotor.RunMode.RUN_TO_POSITION);//Change if you change how they function
                String2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                String.setPower(-StringSpeed);
                String2.setPower(-StringSpeed);

                Count = 0;
            }

            if(gamepad2.a && (!String.isBusy() || !String2.isBusy())){
                Target_Height = 0;//Change

                String.setTargetPosition((int) Target_Height);
                String2.setTargetPosition((int) Target_Height);

                String.setMode(DcMotor.RunMode.RUN_TO_POSITION);//Change if you change how they function
                String2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                String.setPower(-StringSpeed);
                String2.setPower(-StringSpeed);

                Count = 0;
            }

            if(gamepad2.x && (!String.isBusy() || !String2.isBusy())){
                Target_Height = -1000;//Change

                String.setTargetPosition((int) Target_Height);
                String2.setTargetPosition((int) Target_Height);

                String.setMode(DcMotor.RunMode.RUN_TO_POSITION);//Change if you change how they function
                String2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                String.setPower(-StringSpeed);
                String2.setPower(-StringSpeed);

                Count = 0;
            }

            if(gamepad2.b && (!String.isBusy() || !String2.isBusy())){
                Target_Height = -1300;//Change

                String.setTargetPosition((int) Target_Height);
                String2.setTargetPosition((int) Target_Height);

                String.setMode(DcMotor.RunMode.RUN_TO_POSITION);//Change if you change how they function
                String2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                String.setPower(-StringSpeed);
                String2.setPower(-StringSpeed);

                Count = 0;
            }

            Timer--;
            if(gamepad2.right_bumper && Timer < 0){
                Timer = 20;
                Count++;
                if(Count > 4){
                    Count = 0;
                }
            }

            if(gamepad2.left_bumper && (!String.isBusy() || !String2.isBusy())){
                switch ((int) Count){
                    default: {
                        Target_Height = 0;//Change

                        String.setTargetPosition((int) Target_Height);
                        String2.setTargetPosition((int) Target_Height);

                        String.setMode(DcMotor.RunMode.RUN_TO_POSITION);//Change if you change how they function
                        String2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        String.setPower(-StringSpeed);
                        String2.setPower(-StringSpeed);

                        break;
                    }

                    case 1:{
                        Target_Height = -300;//Change

                        String.setTargetPosition((int) Target_Height);
                        String2.setTargetPosition((int) Target_Height);

                        String.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        String2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        String.setPower(-StringSpeed);
                        String2.setPower(-StringSpeed);

                        break;
                    }

                    case 2:{
                        Target_Height = -400;//Change

                        String.setTargetPosition((int) Target_Height);
                        String2.setTargetPosition((int) Target_Height);

                        String.setMode(DcMotor.RunMode.RUN_TO_POSITION);//Change if you change how they function
                        String2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        String.setPower(-StringSpeed);
                        String2.setPower(-StringSpeed);


                        break;
                    }

                    case 3:{
                        Target_Height = -500;//Change

                        String.setTargetPosition((int) Target_Height);
                        String2.setTargetPosition((int) Target_Height);

                        String.setMode(DcMotor.RunMode.RUN_TO_POSITION);//Change if you change how they function
                        String2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        String.setPower(-StringSpeed);
                        String2.setPower(-StringSpeed);


                        break;
                    }


                    case 4:{
                        Target_Height = -600;//Change

                        String.setTargetPosition((int) Target_Height);
                        String2.setTargetPosition((int) Target_Height);

                        String.setMode(DcMotor.RunMode.RUN_TO_POSITION);//Change if you change how they function
                        String2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        String.setPower(-StringSpeed);
                        String2.setPower(-StringSpeed);

                        break;
                    }
                }
            }


            if (!String.isBusy() && !String2.isBusy()){
                String.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                String2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                String.setPower(-0.2);
                String2.setPower(-0.2);
            }
            
            }
            else{

                String.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                String2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                if(gamepad2.y && Timer == 0){
                    Timer = 20;
                }

                if(Timer > 0){
                    String.setPower(-1);
                    String2.setPower(-1);

                    Timer--;
                }else if(gamepad2.right_bumper){
                    String.setPower(0.2);
                    String2.setPower(0.2);
                }
                else if(gamepad2.left_bumper){
                    String.setPower(0);
                    String2.setPower(0);
                }
                else{
                    String.setPower(-0.2);
                    String2.setPower(-0.2);
                }
            }

            if(colors.blue() > 75){
                    telemetry.addLine("         ");
                    telemetry.addLine("          Blue");
                    telemetry.addLine("         ");
                    gamepad2.rumble(0.5, 0.5, 200);  // 200 mSec burst on left motor.
            }else if(colors.red() > 75){
                telemetry.addLine("         ");
                telemetry.addLine("          Red");
                telemetry.addLine("         ");
                gamepad2.rumble(0.5, 0.5, 200);
            }else{
                telemetry.addLine("NoCone");
            }

            //Forebar Motor

            if(gamepad2.dpad_up){
                Turn_Over.setPower(-1);
            }
            else if(gamepad2.dpad_down){
                Turn_Over.setPower(1);
            }
            else if(gamepad2.dpad_right){
                Turn_Over.setPower(0.3);
            }else{
                Turn_Over.setPower(0);
            }


            //Servos
            if(gamepad2.right_trigger > 0){
                ClampPosition += 0.08;
            }
            else{
                ClampPosition = 0.83;
            }

            Clamp.setPosition(ClampPosition);

            //Measure Tape

            if(gamepad2.right_stick_y > 0){
                Rul.setPower(1);
            }else if(gamepad2.right_stick_y < 0) {
                Rul.setPower(-1);
            }else{
                Rul.setPower(0);
            }

            //Updating the telemetry
            telemetry.addLine("Current Hight Target : " + Target_Height);
            telemetry.addLine("Current Stack Cone Number : " + Count);
            telemetry.addLine("Current FR position: " + FR.getCurrentPosition());
            telemetry.addLine("Current FL position: " + FL.getCurrentPosition());
            telemetry.addLine("Current BR position: " + BR.getCurrentPosition());
            telemetry.addLine("Current BL position: " + BL.getCurrentPosition());

            telemetry.addLine("Current String1 position: " + String.getCurrentPosition());
            telemetry.addLine("Current String2 position: " + String2.getCurrentPosition());

            telemetry.addLine("Current OH : " + Turn_Over.getCurrentPosition());

            telemetry.addLine("Servo position" + Clamp.getPosition());


            telemetry.update();
        }

    }



}