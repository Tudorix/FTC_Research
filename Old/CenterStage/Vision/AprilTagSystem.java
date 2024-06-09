package org.firstinspires.ftc.teamcode.CenterStage.Vision;

import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

import java.util.ArrayList;

@Config
public class AprilTagSystem {
    
    private static AprilTagSystem aprilTagSystem = null;
    HardwareMap hardwareMap = null;
    int Tag;
    private boolean running = false;
    Thread AprilTagSystem = null;
    AprilTagProcessor aprilTagProcessor = null;
    
    VisionPortal visionPortal = null;
    
    private AprilTagSystem(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }
    
    public void stop(){
        this.visionPortal.stopStreaming();
    }
    
    public void start(){
        //INIT
        aprilTagProcessor = new AprilTagProcessor.Builder()
                                    .setDrawAxes(true)
                                    .setDrawCubeProjection(true)
                                    .setDrawTagID(true)
                                    .setDrawTagOutline(true)
                                    .build();
        
        visionPortal = new VisionPortal.Builder()
                               .addProcessor(aprilTagProcessor)
                               .setCamera(hardwareMap.get(WebcamName.class , "Webcam 1"))
                               .setCameraResolution(new Size(640 , 360))
                               .build();
    }
    
    public int SeeTag(){
        if(aprilTagProcessor.getDetections().size() > 0){
            AprilTagDetection tag = aprilTagProcessor.getDetections().get(0);
            Tag = tag.id;
        }
        else{
            Tag = 0;
        }
        
        return Tag;
    }
    
    public static synchronized AprilTagSystem getInstance(HardwareMap hardwareMap){
        if(aprilTagSystem == null){
            aprilTagSystem = new AprilTagSystem(hardwareMap);
        }
        return aprilTagSystem;
    }
}
