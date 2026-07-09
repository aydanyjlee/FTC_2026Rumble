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
    public static double P = 100.0;
    public static double I = 0.0;
    public static double D = 1.0;
    public static double F = 15.7;
    public static double Velocity_deg_per_sec = 200;
    public static double intakeMotorPower = 0.8;
    public static double indexerMotorPower = 0.8;
    public static double indexerReverseMotorPower = 0.8;
}

@TeleOp(name = "turretPIDFTune", group = "Tuning")
public class turretPIDFTune extends LinearOpMode {
    RobotHardware robot = new RobotHardware();

    private final FtcDashboard dash = FtcDashboard.getInstance();

    private boolean lastButtonState = false; // Used for "button debouncing"
    private double targetDegrees = 0.0;

    private boolean enableIntake = false;
    private boolean enableIndexer = false;
    private boolean reverseIndexer = false;

    // Hood adjustment vars
    double l = 0.0, r = 0.0;
    boolean lastLeft = false, lastRight = false;
    double stepSize = 0.05;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, RobotHardware.Alliance.RED);
        MultipleTelemetry telemetry = new MultipleTelemetry(this.telemetry, dash.getTelemetry());

//      get motors
        DcMotorEx leftShooter = hardwareMap.get(DcMotorEx.class, "shooterL");
        DcMotorEx rightShooter = hardwareMap.get(DcMotorEx.class, "shooterR");
        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx indexerMotor = hardwareMap.get(DcMotorEx.class, "indexer");

//      Run using encoder
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        rightShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        indexerMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Hood init
        l = robot.hoodL.getPosition();
        r = robot.hoodR.getPosition();

        telemetry.addData("Status", "Initialized. Press Play.");
        telemetry.update();

        waitForStart();

        // Main TeleOp Loop
        while (opModeIsActive()) {
            // --- Hood Adjustment ---
            if (gamepad1.right_bumper && !lastLeft) {
                l = Math.max(0.0, l - stepSize);
                r = Math.min(1.0, r + stepSize);
                robot.hoodL.setPosition(l);
                robot.hoodR.setPosition(r);
            }
            if (gamepad1.right_trigger > 0.9 && !lastRight) {
                l = Math.min(1.0, l + stepSize);
                r = Math.max(0.0, r - stepSize);
                robot.hoodL.setPosition(l);
                robot.hoodR.setPosition(r);
            }
            lastLeft = gamepad1.dpad_left;
            lastRight = gamepad1.dpad_right;

            if (gamepad1.triangle) {
                leftShooter.setVelocity(TuningConfig.Velocity_deg_per_sec, AngleUnit.DEGREES);
                rightShooter.setVelocity(TuningConfig.Velocity_deg_per_sec, AngleUnit.DEGREES);
            }

            if (gamepad1.cross) {
                leftShooter.setVelocity(0);
                rightShooter.setVelocity(0);
            }

            if (gamepad1.circle) {
                leftShooter.setVelocityPIDFCoefficients(TuningConfig.P, TuningConfig.I, TuningConfig.D,TuningConfig.F);
                rightShooter.setVelocityPIDFCoefficients(TuningConfig.P, TuningConfig.I, TuningConfig.D,TuningConfig.F);
            }

            if (gamepad1.leftBumperWasPressed()) {
                enableIntake = !enableIntake;
            }

            // Indexer Forward/Reverse/Stop button handling
            if (gamepad1.dpad_up) { // Forward
                enableIndexer = true;
                reverseIndexer = false;
            } else if (gamepad1.dpad_down) { // Reverse
                enableIndexer = false;
                reverseIndexer = true;
            } else if (gamepad1.dpad_right) { // Stop
                enableIndexer = false;
                reverseIndexer = false;
            }

            // Intake logic only controls intakeMotor
            if (enableIntake) {
                intakeMotor.setPower(TuningConfig.intakeMotorPower);
            } else {
                intakeMotor.setPower(0);
            }

            // Indexer logic only controls indexerMotor
            if (enableIndexer) {
                indexerMotor.setPower(TuningConfig.indexerMotorPower);
            } else if (reverseIndexer) {
                indexerMotor.setPower(-TuningConfig.indexerReverseMotorPower);
            } else {
                indexerMotor.setPower(0);
            }

            telemetry.addData("shooter_target_velocity", TuningConfig.Velocity_deg_per_sec);
            telemetry.addData("left shooter current deg per sec", leftShooter.getVelocity(AngleUnit.DEGREES));
            telemetry.addData("right shooter current deg per sec", rightShooter.getVelocity(AngleUnit.DEGREES));
            telemetry.addData("left_power", leftShooter.getPower());
            telemetry.addData("right_power", rightShooter.getPower());
            telemetry.addData("left_shooter_pidf", leftShooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            telemetry.addData("right_shooter_pidf", rightShooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            // Hood telemetry
            telemetry.addData("HoodL Pos", robot.hoodL.getPosition());
            telemetry.addData("HoodR Pos", robot.hoodR.getPosition());
            telemetry.update();
        }
    }
}
