package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Random;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        final double tileSize = 24.2;

        Random random = new Random();
        final double randomZone = Math.toRadians((random.nextInt(3)-1)*90);

        RoadRunnerBotEntity boardSideBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        //In roadrunner setam pozitia initiala (0,0) si calculam de acolo tot

                        // -------- PUNJABI 2+2 HUNGARIAN FELLAS METHOD --------
                        drive.trajectorySequenceBuilder(new Pose2d(tileSize/2, 60, Math.toRadians(180)))

                                .lineTo(new Vector2d(tileSize/2, 34))
                                .lineTo(new Vector2d(tileSize/2, 60))
                                .splineTo(new Vector2d(44, 38), Math.toRadians(-90))
                                .waitSeconds(1)

                                //to second stack
                                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(180)), Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-tileSize*2, tileSize/2, Math.toRadians(180)), Math.toRadians(180))
                                .forward(20)
                                .back(20)
                                .waitSeconds(0.5)
                                //back to board
                                .splineToLinearHeading(new Pose2d(-tileSize*1.5, 34, Math.toRadians(0)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(44, 38, Math.toRadians(0)), Math.toRadians(0))
                                .build()
                );

        RoadRunnerBotEntity planeSideBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-tileSize*1.5, 60, Math.toRadians(270)))
                                //place TSE
                                .lineTo(new Vector2d(-tileSize*1.5, 34))
                                .lineTo(new Vector2d(-tileSize*1.5, 60))
                                .turn(randomZone)
                                //go to board and score
                                .strafeTo(new Vector2d(44, 38))
                                .turn(-randomZone+1.57) //90degtorad
                                .waitSeconds(1)
                                //go to stack
                                .splineToLinearHeading(new Pose2d(-tileSize*2, tileSize/2, Math.toRadians(180)), Math.toRadians(180))
                                .forward(20)
                                .back(20)
                                .waitSeconds(0.5)
                                //back to the board
                                .splineToLinearHeading(new Pose2d(-tileSize*1.5, 34, Math.toRadians(0)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(44, 38, Math.toRadians(0)), Math.toRadians(0))
                                .build()
                );

        RoadRunnerBotEntity scanPlusPreloadRight = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(tileSize/2, 60, Math.toRadians(270)))
                                .lineTo(new Vector2d(tileSize/2, 34))
                                .splineToLinearHeading(new Pose2d(tileSize/2, 50),Math.toRadians(-180))
                                .splineTo(new Vector2d(44, 38), Math.toRadians(0))
                                .waitSeconds(1)
                                .build()
                );
            
        
        //BOTH FOR FINAL BOT, NOT USING POSITIONING FOR DEVA
        RoadRunnerBotEntity pixelBlue = new DefaultBotBuilder(meepMeep)
                                                .setConstraints(60, 60, Math.toRadians(270), Math.toRadians(270), 15)
                                                .followTrajectorySequence(drive->
                                                drive.trajectorySequenceBuilder(new Pose2d(11.80, 62.00, Math.toRadians(270.00)))
                                                        .splineTo(new Vector2d(11.80, 33.31), Math.toRadians(270.00))
                                                        .lineTo(new Vector2d(34.68, 59.89))
                                                        .splineToLinearHeading(new Pose2d(48.22, 33.50), Math.toRadians(0))
                                                        .build()
    
                                                );
    
        RoadRunnerBotEntity pixelRed = new DefaultBotBuilder(meepMeep)
                                                .setConstraints(60, 60, Math.toRadians(270), Math.toRadians(270), 15)
                                                .followTrajectorySequence(drive->
                                                         drive.trajectorySequenceBuilder(new Pose2d(11.42, -63.88, Math.toRadians(90.00)))
                                                                 .splineTo(new Vector2d(11.42, -38.03), Math.toRadians(90.41))
                                                                 .splineTo(new Vector2d(35.95, -63.51), Math.toRadians(-46.28))
                                                                 .splineTo(new Vector2d(47.65, -36.90), Math.toRadians(0.00))
                                                                 .build()
                                                );

                        

        RoadRunnerBotEntity parkBotRight= new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-tileSize*2, 60, Math.toRadians(270)))
                                .strafeRight(tileSize)
                                .build()
                );

        RoadRunnerBotEntity parkBotLeft = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(tileSize/2, 60, Math.toRadians(270)))
                                .strafeLeft(tileSize*2)
                                .build()
                );




        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setTheme(new ColorSchemeBlueDark())
                .addEntity(pixelRed)
                .start();
    }
}