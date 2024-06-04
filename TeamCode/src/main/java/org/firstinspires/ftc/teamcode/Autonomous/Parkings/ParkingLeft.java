package org.firstinspires.ftc.teamcode.Autonomous.Parkings;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Autonomous
public class ParkingLeft extends LinearOpMode {

    SampleMecanumDrive drive = null;
    Trajectory parkLeftTrajectory = null;
    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);

        parkLeftTrajectory =  drive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .strafeLeft(AutonomousConstants.parkLeftDistance)
                .build();


        waitForStart();
        if(isStopRequested()) {return;}

        drive.followTrajectory(parkLeftTrajectory);



    }
}
