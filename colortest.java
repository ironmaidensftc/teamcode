package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by IM on 9/16/2017.
 */

@Autonomous (name = "color test", group = "otter")
@Disabled

public class colortest extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor leftFrontMotor;
    public DcMotor rightFrontMotor;
    public DcMotor leftRearMotor;
    public DcMotor rightRearMotor;
    public DcMotor rightSlide;

    public Servo lServo;
    public Servo rServo;
    public Servo arm;

    public ColorSensor color;

    @Override
    public void runOpMode() throws InterruptedException {
        double div = 0.75;

        // Define and initialize motors, servos, and sensors
        leftFrontMotor = hardwareMap.dcMotor.get("left_front_drive");
        rightFrontMotor = hardwareMap.dcMotor.get("right_front_drive");
        leftRearMotor = hardwareMap.dcMotor.get("left_rear_drive");
        rightRearMotor = hardwareMap.dcMotor.get("right_rear_drive");
        rightSlide = hardwareMap.dcMotor.get("right_slide");

        lServo = hardwareMap.servo.get("leftServo");
        rServo = hardwareMap.servo.get("rightServo");
        arm = hardwareMap.servo.get("arm");

        color = hardwareMap.get(ColorSensor.class, "color");

        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double SCALE_FACTOR = 255;

        //set the right motors' directions to reverse
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //tell the wheels to run based off the encoders
        idle();

        // Set all motors to zero power
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);


        // Wait for the game to start
        waitForStart();


        while (opModeIsActive()) {
            Color.RGBToHSV((int) (color.red() * SCALE_FACTOR), (int) (color.green() * SCALE_FACTOR), (int) (color.blue() * SCALE_FACTOR), hsvValues);

            //lServo.setPosition(0.7);
            //rServo.setPosition(0.7);

            //sleep(150);

            //arm.setPosition(0.35);

            //sleep(150);

            while(opModeIsActive()) {
                telemetry.addLine("red: " + color.red());
                telemetry.addLine("blue: " + color.blue());
                telemetry.addLine("green: " + color.green());
                telemetry.update();
            }

        }
    }
}
