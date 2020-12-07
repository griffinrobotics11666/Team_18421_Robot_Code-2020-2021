package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Starter Stack Detection", group="Debugging")
public class BotStackDetectionTest extends LinearOpMode {
    Bot bot = new Bot(hardwareMap);
    @Override
    public void runOpMode() throws InterruptedException {
//        bot.initVision(hardwareMap);
//        bot.activateVision();
        bot.detectStarterStack();
        Trajectory b = bot.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(10.0,0.0),0.0)
                .addDisplacementMarker(()->{
                    bot.detectStarterStack();
                })
                .splineTo(new Vector2d(10.0,0.0),Math.toRadians(90.0))
                .build();
        bot.followTrajectory(b);
//        bot.deactivateVision();
    }
}
