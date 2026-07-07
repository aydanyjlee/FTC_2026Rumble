package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.concurrent.ThreadLocalRandom;

@Config
class TuningConfig {
    public static double P = 0.0;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 0.0;
    public static double Velocity_deg_per_sec = 200;
    public static double intakeMotorPower = 0.8;

    public static double indexerMotorPower = 0.8;
}

@TeleOp(name = "turretPIDFTune", group = "Tuning")
public class turretPIDFTune extends LinearOpMode {
    RobotHardware robot = new RobotHardware();

    private final FtcDashboard dash = FtcDashboard.getInstance();

    private boolean lastButtonState = false; // Used for "button debouncing"
    private double targetDegrees = 0.0;

    private boolean enableIntake = false;

    @Override
    public void runOpMode() {

        MultipleTelemetry telemetry = new MultipleTelemetry(this.telemetry, dash.getTelemetry());

//      get motors
        DcMotorEx leftShooter = hardwareMap.get(DcMotorEx.class, "shooterL"); //Shooter motor 1
        DcMotorEx rightShooter = hardwareMap.get(DcMotorEx.class, "shooterR"); //Shooter motor 2
        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx indexerMotor = hardwareMap.get(DcMotorEx.class, "indexer");

//      Run using encoder
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // *** CHANGE DIRECTION IF NEEDED
        leftShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        rightShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        indexerMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized. Press Play.");
        telemetry.update();

        waitForStart();

        // Main TeleOp Loop
        while (opModeIsActive()) {

            if (gamepad1.cross) {
                leftShooter.setVelocity(0);
                rightShooter.setVelocity(0);
            }

            if (gamepad1.triangle) {
                leftShooter.setVelocity(TuningConfig.Velocity_deg_per_sec, AngleUnit.DEGREES);
                rightShooter.setVelocity(TuningConfig.Velocity_deg_per_sec, AngleUnit.DEGREES);
            }

            if (gamepad1.circle) {
                leftShooter.setVelocityPIDFCoefficients(TuningConfig.P, TuningConfig.I, TuningConfig.D,TuningConfig.F);
                rightShooter.setVelocityPIDFCoefficients(TuningConfig.P, TuningConfig.I, TuningConfig.D,TuningConfig.F);
            }            

            if (gamepad1.leftBumperWasPressed()) {
                enableIntake = !enableIntake;
            }

            if (gamepad1.dpadUpWasPressed()){
                indexerMotor.setPower(TuningConfig.indexerMotorPower);
            }
            if (gamepad1.dpadDownWasPressed()){
                indexerMotor.setPower(0);
            }

            if (enableIntake) {
                intakeMotor.setPower(TuningConfig.intakeMotorPower);
            } else {
                intakeMotor.setPower(0);
            }

            telemetry.addData("shooter_target_velocity", TuningConfig.Velocity_deg_per_sec);
            telemetry.addData("left_shooter_current_deg_per_sec", leftShooter.getVelocity(AngleUnit.DEGREES));
            telemetry.addData("right_shooter_current_deg_per_sec", rightShooter.getVelocity(AngleUnit.DEGREES));
            telemetry.addData("left_power", leftShooter.getPower());
            telemetry.addData("right_power", rightShooter.getPower());
            telemetry.addData("PIDF_config", leftShooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            telemetry.update();
        }
    }
}