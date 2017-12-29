package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;






//for red front






@Autonomous(name="block score w vuforia")
@Disabled
public class blockscore extends LinearOpMode {

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
    public BNO055IMU imu;

    VuforiaLocalizer vuforia;

    @Override public void runOpMode() {

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

        Orientation angles;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double SCALE_FACTOR = 255;

        //set the right motors' directions to reverse
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);
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

        VuforiaLocalizer.Parameters parameters2 = new VuforiaLocalizer.Parameters();

        parameters2.vuforiaLicenseKey = " AQJpVY3/////AAAAGb1WwOdpz0Qsv+OxLxlv3M5rYu7Y4Nv+cGXwcVrj8z+EGOPNeBFEiUUMtFdtNBz1sp5pWiEvPhpptm7qrbOCQH75JQ2OZYoMUerUOt0UDTXPyuY+NuUxZojh93tUYht5b6lzOKNwAuxhdfhL/vtGBpcXQQ2oCM/aeMyORQkK2FZsBsG0lIxXLpuEDX8O+Ng1hjhT1KoZhEYZxvXeNrblvgTU9JG7vvgDkR/Fs7F/0tNs8410ikmxY3kpmnLpIwabJ/ChYaCF4kR+cLzkjLvWPso/kGOuPvM9dlV+RSiHNjhne/1VbaNwGUnxCvO9LeKRdlfQOSzk1AqyLULSyxgrpLb2VZz0dVrDYxSddu9II5oh ";

        parameters2.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters2);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();




        // Wait for the game to start
        waitForStart();




        relicTrackables.activate();

        while (opModeIsActive()) {

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

            if (vuMark == RelicRecoveryVuMark.UNKNOWN){
                leftFrontMotor.setPower(0.2);
                leftRearMotor.setPower(0.2);
                rightFrontMotor.setPower(0.2);
                rightRearMotor.setPower(0.2);

                sleep(1500);

                leftFrontMotor.setPower(0);
                leftRearMotor.setPower(0);
                rightFrontMotor.setPower(0);
                rightRearMotor.setPower(0);

                sleep(150);

                leftFrontMotor.setPower(0.2);
                leftRearMotor.setPower(0.2);
                rightFrontMotor.setPower(-0.2);
                rightRearMotor.setPower(-0.2);

                sleep(1150);

                leftFrontMotor.setPower(0);
                leftRearMotor.setPower(0);
                rightFrontMotor.setPower(0);
                rightRearMotor.setPower(0);

                sleep(150);

                leftFrontMotor.setPower(0.2);
                leftRearMotor.setPower(0.2);
                rightFrontMotor.setPower(0.2);
                rightRearMotor.setPower(0.2);

                sleep(450);

                leftFrontMotor.setPower(0);
                leftRearMotor.setPower(0);
                rightFrontMotor.setPower(0);
                rightRearMotor.setPower(0);

                sleep(150);

                leftFrontMotor.setPower(-0.2);
                leftRearMotor.setPower(-0.2);
                rightFrontMotor.setPower(-0.2);
                rightRearMotor.setPower(-0.2);

                sleep(450);
            }
            else if (vuMark == RelicRecoveryVuMark.LEFT){
                leftFrontMotor.setPower(0.2);
                leftRearMotor.setPower(0.2);
                rightFrontMotor.setPower(0.2);
                rightRearMotor.setPower(0.2);

                sleep(1750);

                leftFrontMotor.setPower(0);
                leftRearMotor.setPower(0);
                rightFrontMotor.setPower(0);
                rightRearMotor.setPower(0);

                sleep(150);

                leftFrontMotor.setPower(0.2);
                leftRearMotor.setPower(0.2);
                rightFrontMotor.setPower(-0.2);
                rightRearMotor.setPower(-0.2);

                sleep(1150);

                leftFrontMotor.setPower(0);
                leftRearMotor.setPower(0);
                rightFrontMotor.setPower(0);
                rightRearMotor.setPower(0);

                sleep(150);

                leftFrontMotor.setPower(0.2);
                leftRearMotor.setPower(0.2);
                rightFrontMotor.setPower(0.2);
                rightRearMotor.setPower(0.2);

                sleep(450);

                leftFrontMotor.setPower(0);
                leftRearMotor.setPower(0);
                rightFrontMotor.setPower(0);
                rightRearMotor.setPower(0);

                sleep(150);

                leftFrontMotor.setPower(-0.2);
                leftRearMotor.setPower(-0.2);
                rightFrontMotor.setPower(-0.2);
                rightRearMotor.setPower(-0.2);

                sleep(450);
            }
            else if (vuMark == RelicRecoveryVuMark.RIGHT){
                leftFrontMotor.setPower(0.2);
                leftRearMotor.setPower(0.2);
                rightFrontMotor.setPower(0.2);
                rightRearMotor.setPower(0.2);

                sleep(1250);

                leftFrontMotor.setPower(0);
                leftRearMotor.setPower(0);
                rightFrontMotor.setPower(0);
                rightRearMotor.setPower(0);

                sleep(150);

                leftFrontMotor.setPower(0.2);
                leftRearMotor.setPower(0.2);
                rightFrontMotor.setPower(-0.2);
                rightRearMotor.setPower(-0.2);

                sleep(1150);

                leftFrontMotor.setPower(0);
                leftRearMotor.setPower(0);
                rightFrontMotor.setPower(0);
                rightRearMotor.setPower(0);

                sleep(150);

                leftFrontMotor.setPower(0.2);
                leftRearMotor.setPower(0.2);
                rightFrontMotor.setPower(0.2);
                rightRearMotor.setPower(0.2);

                sleep(450);

                leftFrontMotor.setPower(0);
                leftRearMotor.setPower(0);
                rightFrontMotor.setPower(0);
                rightRearMotor.setPower(0);

                sleep(150);

                leftFrontMotor.setPower(-0.2);
                leftRearMotor.setPower(-0.2);
                rightFrontMotor.setPower(-0.2);
                rightRearMotor.setPower(-0.2);

                sleep(450);
            }
            else if (vuMark == RelicRecoveryVuMark.CENTER) {
                leftFrontMotor.setPower(0.2);
                leftRearMotor.setPower(0.2);
                rightFrontMotor.setPower(0.2);
                rightRearMotor.setPower(0.2);

                sleep(1500);

                leftFrontMotor.setPower(0);
                leftRearMotor.setPower(0);
                rightFrontMotor.setPower(0);
                rightRearMotor.setPower(0);

                sleep(150);

                leftFrontMotor.setPower(0.2);
                leftRearMotor.setPower(0.2);
                rightFrontMotor.setPower(-0.2);
                rightRearMotor.setPower(-0.2);

                sleep(1150);

                leftFrontMotor.setPower(0);
                leftRearMotor.setPower(0);
                rightFrontMotor.setPower(0);
                rightRearMotor.setPower(0);

                sleep(150);

                leftFrontMotor.setPower(0.2);
                leftRearMotor.setPower(0.2);
                rightFrontMotor.setPower(0.2);
                rightRearMotor.setPower(0.2);

                sleep(450);

                leftFrontMotor.setPower(0);
                leftRearMotor.setPower(0);
                rightFrontMotor.setPower(0);
                rightRearMotor.setPower(0);

                sleep(150);

                leftFrontMotor.setPower(-0.2);
                leftRearMotor.setPower(-0.2);
                rightFrontMotor.setPower(-0.2);
                rightRearMotor.setPower(-0.2);

                sleep(450);
            }

            leftFrontMotor.setPower(0);
            leftRearMotor.setPower(0);
            rightFrontMotor.setPower(0);
            rightRearMotor.setPower(0);

            /*
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                telemetry.addData("VuMark", "%s visible", vuMark);
            }
            else {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();*/
        }
    }
}
