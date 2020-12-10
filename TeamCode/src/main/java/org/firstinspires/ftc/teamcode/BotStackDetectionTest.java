package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Starter Stack Detection", group="Debugging")
public class BotStackDetectionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Bot bot = new Bot(hardwareMap);
//        bot.initVision(hardwareMap);
//        bot.activateVision();
        telemetry.setAutoClear(false);
        waitForStart();
        bot.initVision();
        telemetry.addData("Stack detection time","");
        telemetry.update();
        bot.detectStarterStack();
        telemetry.addData("Detected Starter Stack", bot.detectedStack);
        telemetry.update();
//        Trajectory b = bot.trajectoryBuilder(new Pose2d())
//                .splineTo(new Vector2d(10.0,0.0),0.0)
//                .addDisplacementMarker(()->{
//                    telemetry.addData("Displacement marker worked. Detecting stack","");
//                    telemetry.update();
//                    bot.detectStarterStack();
//                    telemetry.addData("Detected Starter Stack", bot.detectedStack);
//                    telemetry.update();
//                })
//                .splineToSplineHeading(new Pose2d(12.0,0.0,Math.toRadians(90)),0.0)
//                .addDisplacementMarker(()->{
//                    telemetry.addData("it turned, yo", "");
//                    telemetry.update();
//                })
//                .build();
//        bot.followTrajectory(b);
        telemetry.addData("Done with stack detection. Deactivating vision","");
        telemetry.update();
        while(opModeIsActive()){
            bot.update();
        }
        bot.deactivateVision();
//        bot.deactivateVision();
    }
}
