package com.ftc23684.meepmeeptesting;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        int square = (70/3);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-square, -(70- (double) (square) /2), Math.toRadians(90)))
                        .splineTo(new Vector2d(square*0.25, square*-1.5), Math.toRadians(90))
                        .splineTo(new Vector2d(square*1, square*-1.5), Math.toRadians(0))
                        .splineTo(new Vector2d(square*2, square*-0.5), Math.toRadians(-90))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}