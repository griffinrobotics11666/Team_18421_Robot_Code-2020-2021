package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Autonomous")
public class BotAutonomous extends LinearOpMode {
    private static Pose2d initialPose = new Pose2d(-63, -25, Math.toRadians(0));

    private static double triggerStart = 0.34;
    private static double triggerEnd = 0.1;
    private static double armDown = 0.34;
    private static double armUp = 0.9;

    private Pose2d A = new Pose2d(12,-40, Math.toRadians(180));
    private Pose2d B = new Pose2d(38, -48, Math.toRadians(0));
    private Pose2d C = new Pose2d(46, -56, Math.toRadians(-90));

    private Pose2d wobbleSpot;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setAutoClear(false);
        telemetry.addData("OpMode Started", "");
        telemetry.update();
        Bot drive = new Bot(hardwareMap);
        drive.usingVuforia(false);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.setPoseEstimate(initialPose);
        drive.initVision();

        drive.Arm.setPosition(armUp);
        drive.Trigger.setPosition(triggerStart);

        Trajectory followStack = drive.trajectoryBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(-45,-33),0)
                .build();
//        Trajectory phase1 = drive.trajectoryBuilder(initialPose)
//                .splineToSplineHeading(new Pose2d(-24, -56, Math.toRadians(-90)), 0.0)
//                .build();
//        Trajectory A = drive.trajectoryBuilder(phase1.end())
//                .splineToSplineHeading(new Pose2d(12,-47, Math.toRadians(180)), 0.0)
//                .build();
//        Trajectory B = drive.trajectoryBuilder(phase1.end())
//                .splineToSplineHeading(new Pose2d(36, -50, Math.toRadians(180)), 0.0)
//                .build();
//        Trajectory C = drive.trajectoryBuilder(phase1.end())
//                .splineToSplineHeading(new Pose2d(48, -60, Math.toRadians(90)), 0.0)
//                .build();
        telemetry.addData("Ready!", "");
        telemetry.update();
        while(!isStarted()) {
            drive.detectStarterStack(1);
        }
        waitForStart();

        drive.Arm.setPosition(0.7);
        sleep(100);
        drive.Arm.setPosition(0.5);
        sleep(200);

        drive.followTrajectory(followStack);
        drive.detectStarterStack(100);
        Trajectory moveWobble = null;
        if(drive.detectedStack == null){
            wobbleSpot = A;
            moveWobble = drive.trajectoryBuilder(followStack.end())
                    .splineToSplineHeading(new Pose2d(-24, -52, Math.toRadians(-90)),0.0)
                    .splineToSplineHeading(wobbleSpot, 0.0)
                    .build();
            drive.followTrajectory(moveWobble);
            drive.Arm.setPosition(armDown);
            sleep(700);
        }
        if(drive.detectedStack == "Single"){
            wobbleSpot = B;
            moveWobble = drive.trajectoryBuilder(followStack.end())
                    .splineToSplineHeading(new Pose2d(-24, -52, Math.toRadians(-90)),0.0)
                    .splineToSplineHeading(wobbleSpot, 0.0)
                    .build();
            drive.followTrajectory(moveWobble);
            drive.Arm.setPosition(armDown);
            sleep(700);
        }
        if(drive.detectedStack == "Quad"){
            wobbleSpot = C;
            drive.usingVuforia(true);
            moveWobble = drive.trajectoryBuilder(followStack.end())
                    .splineToSplineHeading(new Pose2d(-24, -52, Math.toRadians(-90)),0.0)
                    .splineToConstantHeading(new Vector2d(0, -36), 0.0)
                    .splineToSplineHeading(wobbleSpot, 0.0)
                    .build();
            drive.followTrajectory(moveWobble);
            drive.Arm.setPosition(armDown);
            sleep(700);
        }

        telemetry.addData("last Pose", followStack.end());

//        Trajectory moveWobble = drive.trajectoryBuilder(followStack.end())
//                .splineToSplineHeading(new Pose2d(-24, -52, Math.toRadians(-90)),0.0)
//                .splineToSplineHeading(wobbleSpot, 0.0)
//                .build();
//        drive.followTrajectory(moveWobble);
//        drive.clawBase.setPosition(armDown);
//        sleep(700);

        Trajectory phase2 = drive.trajectoryBuilder(new Pose2d(moveWobble.end().getX(),moveWobble.end().getY(),0.0), moveWobble.end().getHeading())
                .splineToSplineHeading(new Pose2d(0, -36, Math.toRadians(-90)), 0.0)
                .build();
        drive.followTrajectory(phase2);

//        Trajectory parkLine = drive.trajectoryBuilder(moveWobble.end())
//                .splineToSplineHeading(new Pose2d(0, 0, 0), 0.0)
//                .build();
//        drive.followTrajectory(parkLine);
        drive.deactivateVision();
    }
}
