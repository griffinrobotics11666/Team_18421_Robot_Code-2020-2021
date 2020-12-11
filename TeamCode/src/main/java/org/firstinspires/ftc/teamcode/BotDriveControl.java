package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.drive.Drive;

import org.firstinspires.ftc.teamcode.Bot;

@Config
@TeleOp(name="Drive Control", group="drive")
public class BotDriveControl extends LinearOpMode{
    private static double triggerStart = 0.34;
    private static double triggerEnd = 0.1;
    private static double armDown = 0.16;
    private static double armUp = 0.6;

    private static boolean fieldCentric = true;

    @Override
    public void runOpMode() throws InterruptedException {
        Bot drive = new Bot(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);
        drive.initVision();

        drive.clawBase.setPosition(armDown);
        drive.Trigger.setPosition(triggerStart);

        waitForStart();
        drive.detectStarterStack(1);

        while(opModeIsActive() && !isStopRequested()){
            //Chassis Drive Code
            if(fieldCentric){
                // Create a vector from the gamepad x/y inputs
                // Then, rotate that vector by the inverse of that heading
                Vector2d input = new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
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
            //Update Cycle
            drive.update();
        }
        drive.deactivateVision();
    }
}

