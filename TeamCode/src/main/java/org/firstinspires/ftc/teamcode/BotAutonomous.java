package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Autonomous")
public class BotAutonomous extends LinearOpMode {
    private static Pose2d initialPose = new Pose2d(-63, -25, Math.toRadians(0));

    private static double triggerStart = 0.34;
    private static double triggerEnd = 0.1;
    private static double armDown = 1;
    private static double armUp = 0;

    private Pose2d A = new Pose2d(-6,-58, Math.toRadians(-90));
    private Pose2d B = new Pose2d(38, -48, Math.toRadians(0));
    private Pose2d C;

    private Pose2d wobbleSpot;
    private ElapsedTime Time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        Bot drive = new Bot(hardwareMap);
        drive.telemetry.addTelemetry(telemetry);
        drive.usingVuforia(false);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.setPoseEstimate(initialPose);
        drive.initVision();

        drive.Arm.setPosition(armUp);
        drive.Trigger.setPosition(triggerStart);
        drive.Latch.setPosition(0.46);

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
        drive.telemetry.addData("Ready!", "");
        drive.telemetry.update();
        while(!isStarted()) {
            drive.detectStarterStack(1);
        }
        waitForStart();
        Time.reset();

        for(double i = 0; i <= 0.5; i+=0.0005){
            drive.Arm.setPosition(i);
        }

        drive.followTrajectory(followStack);
        drive.detectStarterStack(50);
        Trajectory moveWobble = null;
        if(drive.detectedStack == null){
            wobbleSpot = A;
            moveWobble = drive.trajectoryBuilder(followStack.end())
                    .splineToSplineHeading(wobbleSpot, 0.0)
                    .build();
            drive.followTrajectory(moveWobble);
            for(double i = 0.5; i <= 1; i+=0.0005){
                drive.Arm.setPosition(i);
            }
        }
        if(drive.detectedStack == "Single"){
            wobbleSpot = B;
            moveWobble = drive.trajectoryBuilder(followStack.end())
                    .splineToSplineHeading(new Pose2d(-24, -52, Math.toRadians(-90)),0.0)
                    .splineToSplineHeading(wobbleSpot, 0.0)
                    .build();
            drive.followTrajectory(moveWobble);
            for(double i = 0.5; i <= 1; i+=0.0005){
                drive.Arm.setPosition(i);
            }
        }
        if(drive.detectedStack == "Quad"){
//            wobbleSpot = C;
            drive.usingVuforia(true);
            moveWobble = drive.trajectoryBuilder(followStack.end(), Math.toRadians(-90))
                    .splineToSplineHeading(new Pose2d(-45, -37, Math.toRadians(-90)),Math.toRadians(-90))
                    .splineToConstantHeading(new Vector2d(40, -56), 0.0)
                    .build();
            drive.followTrajectory(moveWobble);
            for(double i = 0.5; i <= 1; i+=0.0005){
                drive.Arm.setPosition(i);
            }
            Trajectory letGo = drive.trajectoryBuilder(moveWobble.end())
                    .lineToLinearHeading(new Pose2d(36, -56, 0.0))
                    .build();
            drive.followTrajectory(letGo);
        }

        drive.telemetry.addData("last Pose", followStack.end());

//        Trajectory moveWobble = drive.trajectoryBuilder(followStack.end())
//                .splineToSplineHeading(new Pose2d(-24, -52, Math.toRadians(-90)),0.0)
//                .splineToSplineHeading(wobbleSpot, 0.0)
//                .build();
//        drive.followTrajectory(moveWobble);
//        drive.clawBase.setPosition(armDown);
//        sleep(700);

//        Trajectory phase2 = drive.trajectoryBuilder(new Pose2d(moveWobble.end().getX(),moveWobble.end().getY(),moveWobble.end().getHeading()), 0.0)
//                .lineToLinearHeading(new Pose2d(-0, -40, Math.toRadians(-90)))
//                .build();
//        drive.followTrajectory(phase2);
//        drive.usingVuforia(true);
//        for(int i = 0; i<30; i++){
//            drive.update();
//        }
//        drive.usingVuforia(false);

        /*
        13.23 - 0.83
         */
        Trajectory goShoot = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-7, -34, Math.toRadians(0)))
                .build();
        drive.followTrajectory(goShoot);
        if(drive.batteryVoltageSensor.getVoltage()>=13){
            drive.Shooter.setPower(0.83);
        }
//        else if(drive.batteryVoltageSensor.getVoltage()>=13){
//            drive.Shooter.setPower(0.88);
//        }
        else {
            drive.Shooter.setPower(1.0);
        }
        sleep(2000);
        for(int i = 0; i<3; i++){
            drive.Trigger.setPosition(triggerEnd);
            sleep(500);
            drive.Trigger.setPosition(triggerStart);
            sleep(500);
        }

        Trajectory parkLine = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(18)
                .build();
        drive.followTrajectory(parkLine);
        drive.telemetry.addData("Time this took", Time.seconds());
        drive.telemetry.update();
        //Woohoo!! 56 points!!!
        drive.deactivateVision();
    }
}
