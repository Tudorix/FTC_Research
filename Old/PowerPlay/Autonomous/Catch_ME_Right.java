/*/*
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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@Autonomous
public class Catch_ME_Right extends LinearOpMode
{
    //Robot components
    public DcMotor FL = null;
    public DcMotor FR = null;
    public DcMotor BL = null;
    public DcMotor BR = null;

    public DcMotor String1 = null;
    public DcMotor String2 = null;

    public DcMotor Turn_Over = null;
    public Servo Clamp = null;

    double x = 0 , cos = 0 , sin = 0 , ClampPosition = 0 , ClampPositionLeft = 0, Close = -1;
    double y = 0 , max=0 , FLPower = 0 , FRPower = 0, BLPower = 0 , BRPower = 0;
    double turn = 0 , theta = 0 , power = 0 , Trig_Forc = 0;

    double Speed = 0.5;

    //Camera stuff

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
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
        //Motor Init
        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");

        String1 = hardwareMap.dcMotor.get("LM");
        String2 = hardwareMap.dcMotor.get("LM2");
        Turn_Over = hardwareMap.dcMotor.get("OH");

        Clamp= hardwareMap.servo.get("CL");

        //Motor directions
        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        String2.setDirection(DcMotor.Direction.REVERSE);

        //Cam init
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
                    tagToTelemetry(tagOfInterest);
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
                        tagToTelemetry(tagOfInterest);
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
                    tagToTelemetry(tagOfInterest);
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
            telemetry.addLine("Tag snapshot:" + tagOfInterest);
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest.id == C1){
            Grab(1);
            Movement(3557,3096,3337,3455,Speed);
            Correct(3557,3096,3337,3455,0.2);
            Movement(897 - 150,-931 + 150,-851 + 150,853 - 150,Speed);
            Correct(897 - 150,-931 + 150,-851 + 150,853 - 150,0.2);
            Movement(428,371,459,439,Speed);
            Correct(428,371,459,439,0.2);

            Raise_Lower(-1699,-1705,0.8);
            Hold(1);
            Turn_Arm(-1 , 1000);
            Grab(0);
            try
            {
                Thread.sleep(500);
            }
            catch(InterruptedException e){}
            Turn_Arm(1 , 1000);
            Raise_Lower(1122,1112,0.5);
            Grab(2);
            Hold(1);

            Movement(-331,-383,-350,-381,Speed);
            Correct(-331,-383,-350,-381,0.2);
            Movement(-153,140,-148,138,Speed);
            Correct(-153,140,-148,138,0.2);

            Drive(-0.5,0,600);

            /*
            Grab(1);
            Raise_Lower(-1699 + 600,-1705 + 600,0.8);
            Hold(1);




            Turn_Arm(-1 , 1000);
            Grab(0);
            try
            {
                Thread.sleep(500);
            }
            catch(InterruptedException e){}
            Turn_Arm(1 , 1000);
            Raise_Lower(1122 + 300,1112 ,0.5);
            Grab(2);
            Hold(1);
            */



        }else if(tagOfInterest == null || tagOfInterest.id == C2){
            Grab(1);
            Movement(3557,3096,3337,3455,Speed);
            Correct(3557,3096,3337,3455,0.2);
            Movement(897 - 150,-931 + 150,-851 + 150,853 - 150,Speed);
            Correct(897 - 150,-931 + 150,-851 + 150,853 - 150,0.2);
            Movement(428,371,459,439,Speed);
            Correct(428,371,459,439,0.2);

            Raise_Lower(-1699,-1705,0.8);
            Hold(1);
            Turn_Arm(-1 , 1000);
            Grab(2);
            try
            {
                Thread.sleep(500);
            }
            catch(InterruptedException e){}
            Turn_Arm(1 , 1000);
            Raise_Lower(1122,1112,0.5);
            Grab(2);
            Hold(1);

            Movement(-331,-383,-350,-381,Speed);
            Correct(-331,-383,-350,-381,0.2);
            Movement(-153,140,-148,138,Speed);
            Correct(-153,140,-148,138,0.2);

            Drive(0.5,0,600);

            /*
            Grab(1);
            Raise_Lower(-1699 + 600,-1705 + 600,0.8);
            Hold(1);




            Turn_Arm(-1 , 1000);
            Grab(0);
            try
            {
                Thread.sleep(500);
            }
            catch(InterruptedException e){}
            Turn_Arm(1 , 1000);
            Raise_Lower(1122 + 300,1112 ,0.5);
            Grab(2);
            Hold(1);
            */


        }else if(tagOfInterest.id == C3){
            Grab(1);
            Movement(3557,3096,3337,3455,Speed);
            Correct(3557,3096,3337,3455,0.2);
            Movement(897 - 150,-931 + 150,-851 + 150,853 - 150,Speed);
            Correct(897 - 150,-931 + 150,-851 + 150,853 - 150,0.2);
            Movement(428,371,459,439,Speed);
            Correct(428,371,459,439,0.2);

            Raise_Lower(-1699,-1705,0.8);
            Hold(1);
            Turn_Arm(-1 , 1000);
            Grab(2);
            try
            {
                Thread.sleep(500);
            }
            catch(InterruptedException e){}
            Turn_Arm(1 , 1000);
            Raise_Lower(1122,1112,0.5);
            Grab(2);
            Hold(1);

            Movement(-331,-383,-350,-381,Speed);
            Correct(-331,-383,-350,-381,0.2);
            Movement(-153,140,-148,138,Speed);
            Correct(-153,140,-148,138,0.2);

            Drive(0.5,0,2200);
        }
        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    void Movement(int FRp,int FLp,int BRp , int BLp , double Power){
        //Reset encoders
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set target position
        FL.setTargetPosition(FLp);
        FR.setTargetPosition(FRp);
        BL.setTargetPosition(BLp);
        BR.setTargetPosition(BRp);

        //Set motors to run to position
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set power to all motors
        FL.setPower(Power);
        FR.setPower(Power);
        BL.setPower(Power);
        BR.setPower(Power);

        while(FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy()){
            //This waits until target position is reached
            telemetry.addLine("Working...");
        }

        //Stop motors
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Done!");
    }

    void Raise_Lower(int S1p , int S2p , double Power){

        String1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        String2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        String1.setTargetPosition(S1p);
        String2.setTargetPosition(S2p);

        String1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        String2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        String1.setPower(Power);
        String2.setPower(Power);

        while(String1.isBusy() && String2.isBusy()){
            //Working on it :)
        }

        String1.setPower(0);
        String2.setPower(0);

        String1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        String2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void Hold(int Case){
        if(Case > 0){
            String1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            String2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            String1.setPower(-0.2);
            String2.setPower(-0.2);
        }else{
            String1.setPower(0);
            String2.setPower(0);

            String1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            String2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    void Grab(int C){
        if(C == 0){
            Clamp.setPosition(0.85);//Must change
        }else if(C == 1){
            Clamp.setPosition(1);//Must change
        }else{
            Clamp.setPosition(0.9);//Must change
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

    public void Correct(int FRp,int FLp,int BRp , int BLp , double Power){
        //Set target position
        FL.setTargetPosition(FLp);
        FR.setTargetPosition(FRp);
        BL.setTargetPosition(BLp);
        BR.setTargetPosition(BRp);

        //Set motors to run to position
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set power to all motors
        FL.setPower(Power);
        FR.setPower(Power);
        BL.setPower(Power);
        BR.setPower(Power);

        while(FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy()){
            //This waits until target position is reached
            telemetry.addLine("Working...");
        }

        //Stop motors
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Done!");
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
}
 