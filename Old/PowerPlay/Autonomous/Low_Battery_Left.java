/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.PowerPlay.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class Low_Battery_Left extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!

    //for the movement
    public DcMotor FL = null;
    public DcMotor FR = null;
    public DcMotor BL = null;
    public DcMotor BR = null;

    public DcMotor String = null;
    public DcMotor String2 = null;

    public DcMotor Turn_Over = null;

    public Servo Clamp = null;

    double x = 0 , cos = 0 , sin = 0 , ClampPositionRight = 0 , ClampPositionLeft = 0,Close = -1,Trigger_Force =0 ;
    double y = 0 , max=0 , FLPower = 0 , FRPower = 0, BLPower = 0 , BRPower = 0;
    double turn = 0 , theta = 0 , power = 0;
    long Timer = 0;
    int Mode = -1 , Catch = -1;

    //For the camera
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //Tag IDs of sleeve
    int C1 = 1;
    int C2 = 3;
    int C3 = 6;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");

        String = hardwareMap.dcMotor.get("LM");
        String2 = hardwareMap.dcMotor.get("LM2");
        Turn_Over = hardwareMap.dcMotor.get("OH");

        Clamp = hardwareMap.servo.get("CL");

        //Set direction to all motors

        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        String2.setDirection(DcMotor.Direction.REVERSE);
        Clamp.setDirection(Servo.Direction.FORWARD);

        String.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        String2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == C1 || tag.id == C2 || tag.id == C3)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest.id == C1){
            //Place cone on Low
            Close_Arm(1);
            Drive(0 , 0.5 , 1300);
            Turn(0.5 , 1200);
            Move_LinerMotion(-1, 1500);
            String.setPower(-0.2);
            String2.setPower(-0.2);
            Drive(0 , -0.5 , 400);
            try
            {
                Thread.sleep(500);
            }
            catch(InterruptedException e){}
            Close_Arm(0);
            try
            {
                Thread.sleep(500);
            }
            catch(InterruptedException e){}
            Drive(0 , 0.5 , 400);
            Drive(-0.5 , 0, 1000);
            Drive(0 , -0.5 , 1900);


        }else if(tagOfInterest == null || tagOfInterest.id == C2){
            Close_Arm(1);
            Drive(0 , 0.5 , 1300);
            Turn(0.5 , 1200);
            Move_LinerMotion(-1, 1500);
            String.setPower(-0.2);
            String2.setPower(-0.2);
            Drive(0 , -0.5 , 400);
            try
            {
                Thread.sleep(500);
            }
            catch(InterruptedException e){}
            Close_Arm(0);
            try
            {
                Thread.sleep(500);
            }
            catch(InterruptedException e){}
            Drive(0 , 0.5 , 400);
            Drive(-0.5 , 0, 1000);


        }else if(tagOfInterest.id == C3){
            Close_Arm(1);
            Drive(0 , 0.5 , 1300);
            Turn(0.5 , 1200);
            Move_LinerMotion(-1, 1500);
            String.setPower(-0.2);
            String2.setPower(-0.2);
            Drive(0 , -0.5 , 400);
            try
            {
                Thread.sleep(500);
            }
            catch(InterruptedException e){}
            Close_Arm(0);
            try
            {
                Thread.sleep(500);
            }
            catch(InterruptedException e){}
            Drive(0 , 0.5 , 400);
            Drive(-0.5 , 0, 1000);
            Drive(0 , 0.5 , 1900);
        }

        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }



    public void Drive(double XInput, double YInput, long Time){

        //Multiplier = (-x + y)/5;
        theta = Math.atan2(YInput , XInput);
        power = Math.hypot(XInput, YInput);

        sin = Math.sin(theta - Math.PI/4);
        cos = Math.cos(theta - Math.PI/4);
        max = Math.max(Math.abs(sin) , Math.abs(cos));

        FLPower = power * cos/max;
        FRPower = power * sin/max;
        BLPower = power * sin/max;
        BRPower = power * cos/max;

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

        try
        {
            Thread.sleep(Time);
        }
        catch(InterruptedException e){}

        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);

    }

    public void Turn(double turn,long Time){
        FLPower = turn;
        FRPower = -turn;
        BLPower = turn;
        BRPower = -turn;

        FL.setPower(FLPower);
        FR.setPower(FRPower);
        BL.setPower(BLPower);
        BR.setPower(BRPower);

        try
        {
            Thread.sleep(Time);
        }
        catch(InterruptedException e){}

        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    public void Move_LinerMotion(double Direct ,long Time){
        String.setPower(Direct);
        String2.setPower(Direct);

        try
        {
            Thread.sleep(Time);
        }
        catch(InterruptedException e){}

        String.setPower(0);
        String2.setPower(0);
    }

    public void Close_Arm(double State){
        if(State == 1){
            Clamp.setPosition(1);
        }else if(State == 0){
            Clamp.setPosition(0.68);
        }
    }

    public void Turn_Arm(double direction , long Time){ // -1 = Upwards
        Turn_Over.setPower(direction);

        try
        {
            Thread.sleep(Time);
        }
        catch(InterruptedException e){}

        Turn_Over.setPower(0);
    }

}