package org.firstinspires.ftc.teamcode.Discover.Vision;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

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
