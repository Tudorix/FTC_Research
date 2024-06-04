package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class   TSESystem {

    private static TSESystem tseSystem = null;
    private boolean running = false;
    Thread IntakeSystem = null;
    HardwareMap hardwareMap = null;
    OpenCvCamera camera = null;
    
    int Case;
    private TSESystem(HardwareMap instance){
        hardwareMap = instance;
    }
    
    public void stop(){
        this.camera.stopStreaming();
        this.running = false;
    }
    public void start(String color){
        running = true;
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(camera, 0);
    
        TeamProp_Pipeline teamProp_pipeline = new TeamProp_Pipeline(color);
        camera.setPipeline(teamProp_pipeline);
    
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640 , 360, OpenCvCameraRotation.UPRIGHT);
            }
        
            @Override
            public void onError(int errorCode) {
            
            }
        });
        /*if(IntakeSystem == null || !IntakeSystem.isAlive()){
            IntakeSystem = new Thread(() -> {
                while(running){
                
                }
            });
        }*/
    }
    
    public int SeeCase(){
        return Case;
    }
    
    public static synchronized TSESystem getInstance(HardwareMap hardwareMap){
        if(tseSystem == null){
            tseSystem = new TSESystem(hardwareMap);
        }
        return tseSystem;
    }
    
    class TeamProp_Pipeline extends OpenCvPipeline {
        
        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat midCrop;
        Mat rightCrop;
        double leftAvgFin;
        double midAvgFin;
        double rightAvgFin;
        Mat outPut = new Mat();
        Scalar rectColor = new Scalar(0 , 0, 0);
        
        int channel = 0;
        
        public TeamProp_Pipeline(String text){
            if(text.equals("blue")){
                channel = 2;
            }else if(text.equals("red")){
                channel = 0;
            }
        }
        
        public Mat processFrame(Mat input){
            Imgproc.cvtColor(input, YCbCr , Imgproc.COLOR_RGB2YCrCb);
            // pipeline running
            
            Rect leftRect = new Rect(1 , 1 , 212 , 359);
            Rect midRect = new Rect(213 , 1 , 212 , 359);
            Rect rightRect = new Rect(426 , 1 , 212 , 359);
            
            input.copyTo(outPut);
            Imgproc.rectangle(outPut, leftRect, rectColor, 2);
            Imgproc.rectangle(outPut, midRect, rectColor, 2);
            Imgproc.rectangle(outPut, rightRect, rectColor, 2);
            
            leftCrop = YCbCr.submat(leftRect);
            midCrop = YCbCr.submat(midRect);
            rightCrop = YCbCr.submat(rightRect);
            
            Core.extractChannel(leftCrop , leftCrop , channel);
            Core.extractChannel(midCrop , midCrop , channel);
            Core.extractChannel(rightCrop , rightCrop, channel);
            
            Scalar leftavg = Core.mean(leftCrop);
            Scalar midavg = Core.mean(midCrop);
            Scalar rightavg = Core.mean(rightCrop);
            
            leftAvgFin = leftavg.val[0];
            midAvgFin = midavg.val[0];
            rightAvgFin = rightavg.val[0];
            
            if(Math.max(leftAvgFin , Math.max(midAvgFin , rightAvgFin)) == leftAvgFin){
                //Element on LEFT
                Case = 1;
            }else if(Math.max(leftAvgFin , Math.max(midAvgFin , rightAvgFin)) == midAvgFin){
                //Element on MIDDLE
                Case = 2;
            }else if(Math.max(leftAvgFin , Math.max(midAvgFin , rightAvgFin)) == rightAvgFin){
                //Element on RIGHT
                Case = 3;
            }else{
                Case = 0;
            }
            
            return (outPut);
        }
    }
}
