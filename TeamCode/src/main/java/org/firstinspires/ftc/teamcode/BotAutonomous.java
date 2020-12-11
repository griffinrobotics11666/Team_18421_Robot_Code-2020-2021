package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class BotAutonomous extends LinearOpMode {
    private static Pose2d initialPose = new Pose2d(-63, -25, 0.0);

    private static double triggerStart = 0.34;
    private static double triggerEnd = 0.1;
    private static double armDown = 0.16;
    private static double armUp = 0.6;

    @Override
    public void runOpMode() throws InterruptedException {
        Bot drive = new Bot(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);
        drive.initVision();

        drive.clawBase.setPosition(armUp);
        drive.Trigger.setPosition(triggerStart);

        Trajectory phase1 = drive.trajectoryBuilder(initialPose)
                .splineToSplineHeading(new Pose2d(-24, -56, Math.toRadians(-90)), 0.0)
                .build();
        Trajectory A = drive.trajectoryBuilder(phase1.end())
                .splineToSplineHeading(new Pose2d(12,-47, Math.toRadians(180)), 0.0)
                .build();
        Trajectory B = drive.trajectoryBuilder(phase1.end())
                .splineToSplineHeading(new Pose2d(36, -50, 0.0), 0.0)
                .build();
        Trajectory C = drive.trajectoryBuilder(phase1.end())
                .splineToSplineHeading(new Pose2d(48, -60, Math.toRadians(-90)), 0.0)
                .build();
        while(!isStarted()) {
            drive.detectStarterStack(1);
        }
        waitForStart();

        drive.followTrajectory(phase1);

        if(drive.detectedStack == null){
            drive.followTrajectory(A);
        }
        if(drive.detectedStack == "Single"){
            drive.followTrajectory(B);
        }
        if(drive.detectedStack == "Quad"){
            drive.followTrajectory(C);
        }

        drive.clawBase.setPosition(armDown);
        drive.deactivateVision();
    }
}
