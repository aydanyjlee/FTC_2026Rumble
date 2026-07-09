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
    public static double P = 10.0;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 40;
    public static double Velocity_deg_per_sec = 150;
    public static double intakeMotorPower = 0.8;
    public static double indexerMotorPower = 0.8;
    public static double indexerReverseMotorPower = 0.8;
}

@TeleOp(name = "turretPIDFTune", group = "Tuning")
public class turretPIDFTune extends LinearOpMode {

    private final FtcDashboard dash = FtcDashboard.getInstance();

    private boolean lastButtonState = false;
    private double targetDegrees = 0.0;

    private boolean enableIntake = false;
    private boolean enableIndexer = false;
    private boolean reverseIndexer = false;

    @Override
    public void runOpMode() {
        MultipleTelemetry telemetry = new MultipleTelemetry(this.telemetry, dash.getTelemetry());

//      get motors
        DcMotorEx leftShooter = hardwareMap.get(DcMotorEx.class, "shooterL");  // Rename this to match your configuration
        DcMotorEx rightShooter = hardwareMap.get(DcMotorEx.class, "shooterR"); //
        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");    //
        DcMotorEx indexerMotor = hardwareMap.get(DcMotorEx.class, "indexer");  //
 
//      Run using encoder
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftShooter.setDirection(DcMotorSimple.Direction.FORWARD);   // Check if the direction is correct
        rightShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        indexerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        telemetry.addData("Status", "Initialized. Press Play.");
        telemetry.update();
        waitForStart();

        // Main TeleOp Loop
        while (opModeIsActive()) {

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
                indexerMotor.setPower(TuningConfig.indexerMotorPower);
            } else if (gamepad1.dpad_down) { // Reverse
                indexerMotor.setPower(0);
            } 

            // Intake logic only controls intakeMotor
            if (enableIntake) {
                intakeMotor.setPower(TuningConfig.intakeMotorPower);
            } else {
                intakeMotor.setPower(0);
            }

            telemetry.addData("shooter_target_velocity", TuningConfig.Velocity_deg_per_sec);
            telemetry.addData("left shooter current deg per sec", leftShooter.getVelocity(AngleUnit.DEGREES));
            telemetry.addData("right shooter current deg per sec", rightShooter.getVelocity(AngleUnit.DEGREES));
            telemetry.addData("PIDF_values", leftShooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            
            telemetry.update();
        }
    }
}
