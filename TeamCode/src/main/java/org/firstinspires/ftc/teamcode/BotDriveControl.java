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

    Left Joystick = Drive
    Right Joystick = Turn

    Dpad Left = Reverse Feeder - Only works if feeder is off
    Dpad Down = Quit Path Following
    Dpad Up = Reset Position using Vuforia
    Dpad Right = Reset Heading to 0 (Straight towards the goals)

    Back = Toggle Field Centric Drive

    Left Bumper = Automatically drive to shoot at High Goal

    Left Trigger = Linear Slide down
    Right Trigger = Linear Slide up
     */


    private static double triggerStart = 0.34;
    private static double triggerEnd = 0.1;

    private static boolean fieldCentric = true;
    public static Pose2d highGoal = new Pose2d(-5,-36, Math.toRadians(0));
    public static Pose2d powerShot = new Pose2d(2, -18.5, Math.toRadians(0));
    private boolean arePowerShooting = false;
//    public static Pose2d middlePower = new Pose2d(2, -11.5, Math.toRadians(0));
//    public static Pose2d leftPower = new Pose2d(2, -5.5, Math.toRadians(0));

    private ElapsedTime shootingClock = new ElapsedTime();
    public static double shootingDelay = 300;
    public static double shootingCooldown = 300;
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
    private boolean isBackPressed = false;
    private double linearSlideCoefficient = 1;
    private double shootSpeed = 1;

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
        drive.telemetry.addTelemetry(telemetry);
        drive.usingVuforia(false);

        drive.initVision();
        drive.Arm.setPosition(1);
        drive.Trigger.setPosition(triggerStart);
        drive.Latch.setPosition(0.0);

        drive.telemetry.addData("Ready!", "");
        drive.telemetry.update();
        waitForStart();
        drive.detectStarterStack(1);

        while(opModeIsActive() && !isStopRequested()){
            Pose2d currentPose = drive.getPoseEstimate();
            switch(mode){
                case DRIVER_CONTROL: {
                    //Linear Slide Control
                    drive.linearSlide.setPosition(0.5-(gamepad1.left_trigger*linearSlideCoefficient*0.5)+(gamepad1.right_trigger*linearSlideCoefficient*0.5));

                    //Wobble Cycle
                    if(wobbleMode == 0){
                        drive.Arm.setPosition(1.0);
                    }
                    else if(wobbleMode == 1){
                        drive.Arm.setPosition(0.4);
                    }
                    else if(wobbleMode == 2){
                        drive.Arm.setPosition(0.8);
                    }
                    if(gamepad1.a && !isAPressed){
                        if(wobbleMode == 0){
                            drive.Arm.setPosition(1.0);
                            wobbleMode = 1;
                        }
                        else if(wobbleMode == 1){
                            drive.Arm.setPosition(0.4);
                            wobbleMode = 2;
                        }
                        else if(wobbleMode == 2){
                            drive.Arm.setPosition(0.8);
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
                            drive.Intake.setPower(0.6);
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
                        if(drive.batteryVoltageSensor.getVoltage()>=13){
                            drive.Shooter.setPower(0.83*shootSpeed);
                        }
//        else if(drive.batteryVoltageSensor.getVoltage()>=13){
//            drive.Shooter.setPower(0.88);
//        }
                        else {
                            drive.Shooter.setPower(1.0*shootSpeed);
                        }
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
                                .lineToLinearHeading(highGoal)
                                .build();
                        drive.followTrajectoryAsync(followGoal);
                        mode = Mode.PATH_FOLLOWING;
                    }

                    //Auto PowerShot code
                    if(gamepad1.right_bumper && !arePowerShooting){
                        arePowerShooting = true;
                        shootSpeed = 0.8;
                        Trajectory initialPos = drive.trajectoryBuilder(currentPose)
                                .lineToLinearHeading(powerShot)
                                .build();
                        drive.followTrajectoryAsync(initialPos);
                        mode = Mode.PATH_FOLLOWING;
                    }
                    else if(gamepad1.right_bumper && arePowerShooting){
                        Trajectory adjust = drive.trajectoryBuilder(currentPose)
                                .strafeLeft(7)
                                .build();
                        drive.followTrajectoryAsync(adjust);
                        mode = Mode.PATH_FOLLOWING;
                    }
                    if(gamepad1.left_stick_button){
                        arePowerShooting = false;
                        shootSpeed = 1;
                    }

                    //Reset Position
                    if(gamepad1.dpad_up){
                        drive.getVuforiaPosition = true;
                    }
                    else {
                        drive.getVuforiaPosition = false;
                    }

                    //Reset Heading
                    if(gamepad1.dpad_right){
                        drive.resetHeading();
                    }

                    //Field Centric Drive Toggle
                    if(gamepad1.back && fieldCentric && !isBackPressed) {
                        fieldCentric = false;
                        isBackPressed = true;
                    }
                    else if(gamepad1.back && !fieldCentric && !isBackPressed) {
                        fieldCentric = true;
                        isBackPressed = true;
                    }
                    if(!gamepad1.back){
                        isBackPressed = false;
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
                    break;
                }

            }

            //Update Cycle
            drive.update();
        }
        drive.deactivateVision();
    }
}

