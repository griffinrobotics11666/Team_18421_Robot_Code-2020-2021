package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.drive.Drive;

import org.firstinspires.ftc.teamcode.Bot;

@Config
@TeleOp(name="Drive Control", group="drive")
public class BotDriveControl extends OpMode{
    Bot drive = new Bot(hardwareMap);

    @Override
    public void init() {
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.setPoseEstimate(new Pose2d(0, 0, 0));
//        drive.initVision(hardwareMap);
//        drive.activateVision();
    }

    @Override
    public void loop(){
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );
        drive.update();
    }

//    public void stop(){
//        drive.deactivateVision();
//    }
}

