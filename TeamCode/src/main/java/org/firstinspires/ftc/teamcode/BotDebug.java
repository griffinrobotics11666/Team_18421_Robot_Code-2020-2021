package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;

import org.firstinspires.ftc.teamcode.Bot;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Config
@TeleOp(name="Bot Debug", group="Debugging")
public class BotDebug extends LinearOpMode {
    public static double shooterPower = 0.0;
    public static double intakePower= 0.0;
    public static double ringPushingPosition = 0.34;
    public static double linearSlidePower = 0.5;
    public static double clawBasePosition = 0.5;
    public static double clawPosition = 0.5;
    public static boolean canDrive = true;

    public static double highGoalX = 70.75;
    public static double highGoalZ = -46.5+12;
    public static double highGoalY = 91/2.54;

    public static double shootingHeight = 0.0;

    private double deltaX;
    private double deltaZ;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setAutoClear(false);
        telemetry.addData("initializing bot","");
        telemetry.update();
        Bot bot = new Bot(hardwareMap);
        telemetry.addData("setting runmode","");
        telemetry.update();
        bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("setting pose estimate","");
        telemetry.update();
        bot.setPoseEstimate(new Pose2d(0, -48, 0));
        telemetry.addData("initializing vision","");
        telemetry.update();
        bot.initVision();
        waitForStart();
        bot.detectStarterStack(1);
        telemetry.addData("running while loop","");
        telemetry.update();
        while (opModeIsActive()) {
            deltaX = bot.getPoseEstimate().getX()-highGoalX;
            deltaZ = bot.getPoseEstimate().getY()-highGoalZ;
            double initialvY = Math.sqrt(-2*-9.8*(highGoalY-shootingHeight));
            double vyTime = initialvY/9.8;

            bot.Shooter.setPower(shooterPower);
            bot.Intake.setPower(intakePower);
            bot.Trigger.setPosition(ringPushingPosition);
            bot.linearSlide.setPosition(linearSlidePower);
            bot.clawBase.setPosition(clawBasePosition);
            bot.Claw.setPosition(clawPosition);

            if(canDrive){
                // Create a vector from the gamepad x/y inputs
                // Then, rotate that vector by the inverse of that heading
                Vector2d input = new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                ).rotated(-bot.getPoseEstimate().getHeading());

                // Pass in the rotated input + right stick value for rotation
                // Rotation is not part of the rotated input thus must be passed in separately
                bot.setWeightedDrivePower(
                        new Pose2d(
                                input.getX(),
                                input.getY(),
                                -gamepad1.right_stick_x
                        )
                );
            }
            bot.update();
        }

        bot.deactivateVision();
    }

}
