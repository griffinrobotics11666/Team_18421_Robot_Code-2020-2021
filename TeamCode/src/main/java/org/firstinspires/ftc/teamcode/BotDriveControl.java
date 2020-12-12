package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Bot;

@Config
@TeleOp(name="Drive Control", group="drive")
public class BotDriveControl extends LinearOpMode{
    //Controls
    /*
    A = Toggle Arm Position - Low,High,Middle
    B = Intake Toggle
    X = Shoot Button - Only works if shooter is on
    Y = Shooter Toggle

    Left Joystick = Field Centric Drive
    Right Joystick = Robot Heading

    Dpad Left = Reverse Feeder - Only works if feeder is off
     */


    private static double triggerStart = 0.34;
    private static double triggerEnd = 0.1;

    private static boolean fieldCentric = true;

    public static Pose2d highGoal = new Pose2d(-4,-40, Math.toRadians(12));
    public static Pose2d rightPower = new Pose2d(-4, -25, Math.toRadians(12));
    public static Pose2d middlePower = new Pose2d(-4, -25, Math.toRadians(30));
    public static Pose2d leftPower = new Pose2d(-4, -25, Math.toRadians(45));

    private ElapsedTime shootingClock = new ElapsedTime();
    public static double shootingDelay = 500;
    public static double shootingCooldown = 500;
    private enum ShootingState{
        SHOOT,
        RESET,
        WAIT
    }
    private ShootingState shoot = ShootingState.SHOOT;
    private boolean isShooting = false;
    private boolean isYPressed = false;
    private boolean isFeeding = false;
    private boolean isBPressed = false;
    private int wobbleMode = 0;
    private boolean isAPressed = false;

    private enum Mode {
        DRIVER_CONTROL,
        PATH_FOLLOWING
    }
    private Mode mode = Mode.DRIVER_CONTROL;

    @Override
    public void runOpMode() throws InterruptedException {
        Bot drive = new Bot(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);


        drive.initVision();
        drive.clawBase.setPosition(0.9);
        drive.Trigger.setPosition(triggerStart);
        telemetry.addData("Ready!", "");
        telemetry.update();
        waitForStart();
        drive.detectStarterStack(1);

        while(opModeIsActive() && !isStopRequested()){
            Pose2d currentPose = drive.getPoseEstimate();
            switch(mode){
                case DRIVER_CONTROL: {
                    //Wobble Cycle
                    if(wobbleMode == 0){
                        drive.clawBase.setPosition(0.34);
                    }
                    else if(wobbleMode == 1){
                        drive.clawBase.setPosition(0.69);
                    }
                    else if(wobbleMode == 2){
                        drive.clawBase.setPosition(0.5);
                    }
                    if(gamepad1.a && !isAPressed){
                        if(wobbleMode == 0){
                            drive.clawBase.setPosition(0.34);
                            wobbleMode = 1;
                        }
                        else if(wobbleMode == 1){
                            drive.clawBase.setPosition(0.69);
                            wobbleMode = 2;
                        }
                        else if(wobbleMode == 2){
                            drive.clawBase.setPosition(0.5);
                            wobbleMode = 0;
                        }
                        isAPressed = true;
                    }
                    if(!gamepad1.a){
                        isAPressed = false;
                    }

                    //Intake Toggle
                    if(gamepad1.y && !isYPressed){
                        if(isFeeding){
                            isFeeding = false;
                        }
                        else{
                            isFeeding = true;
                        }
                        isYPressed = true;
                    }
                    if(!gamepad1.y){
                        isYPressed = false;
                    }
                    if(isFeeding){
                        drive.Intake.setPower(-0.3);
                    }
                    else {
                        //Intake Reversal
                        if(gamepad1.dpad_left){
                            drive.Intake.setPower(0.3);
                        }
                        else{
                            drive.Intake.setPower(0);
                        }
                    }


                    //Shooter Toggle
                    if(gamepad1.b && !isBPressed){
                        if(isShooting){
                            isShooting=false;
                        }
                        else{
                            isShooting=true;
                        }
                        isBPressed = true;
                    }
                    if(!gamepad1.b){
                        isBPressed = false;
                    }
                    if(isShooting){
                        drive.Shooter.setPower(1.0);
                    }
                    else {
                        drive.Shooter.setPower(0);
                    }

                    //Shooting Code
                    switch(shoot){
                        case SHOOT: {
                            if(gamepad1.x && isShooting){
                                shootingClock.reset();
                                drive.Trigger.setPosition(triggerEnd);
                                shoot = ShootingState.RESET;
                                break;
                            }
                        }
                        case RESET: {
                            if(shootingClock.milliseconds()>= shootingDelay){
                                shootingClock.reset();
                                drive.Trigger.setPosition(triggerStart);
                                shoot = ShootingState.WAIT;
                                break;
                            }
                        }
                        case WAIT: {
                            if(shootingClock.milliseconds()>= shootingCooldown){
                                shootingClock.reset();
                                shoot=ShootingState.SHOOT;
                                break;
                            }
                        }
                    }

                    //Chassis Drive Code
                    if(fieldCentric){
                        // Create a vector from the gamepad x/y inputs
                        // Then, rotate that vector by the inverse of that heading
                        Vector2d input = new Vector2d(
                                gamepad1.left_stick_x,
                                -gamepad1.left_stick_y
                        ).rotated(-drive.getPoseEstimate().getHeading());

                        // Pass in the rotated input + right stick value for rotation
                        // Rotation is not part of the rotated input thus must be passed in separately
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        input.getX(),
                                        input.getY(),
                                        -gamepad1.right_stick_x
                                )
                        );
                    }
                    else {
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        -gamepad1.left_stick_y,
                                        -gamepad1.left_stick_x,
                                        -gamepad1.right_stick_x
                                )
                        );
                    }

                    //Automatic aim code
                    if(gamepad1.left_bumper){
                        Trajectory followGoal = drive.trajectoryBuilder(currentPose)
                                .splineToSplineHeading(highGoal, 0.0)
                                .build();
                        drive.followTrajectoryAsync(followGoal);
                        mode = Mode.PATH_FOLLOWING;
                    }

                    break;
                }
                case PATH_FOLLOWING: {
                    if(gamepad1.dpad_down){
                        drive.breakTrajectory();
                        mode = Mode.DRIVER_CONTROL;
                    }
                    if(!drive.isBusy()){
                        mode = Mode.DRIVER_CONTROL;
                    }
                }

            }

            //Update Cycle
            drive.update();
        }
        drive.deactivateVision();
    }
}

