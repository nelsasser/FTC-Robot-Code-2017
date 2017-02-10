package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Drives forward and turns to knock ball off
 */
@Autonomous(name="KarelAuto", group="Pushbot")
public class KarelAuto extends LinearOpMode {
    KarelHardware robot = new KarelHardware(); // use the class created to define karel's hardware

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialized, waiting for start");
        telemetry.update();

        waitForStart();

        runtime.reset();
        while(runtime.seconds() < .25) {
            robot.rightMotor.setPower(-.5);
            robot.leftMotor.setPower(-.5);
        }
        robot.rightMotor.setPower(0);
        robot.leftMotor.setPower(0);

        runtime.reset();

        while(runtime.seconds() < 1) {
            robot.launcher.setPower(1);
        }
        robot.launcher.setPower(0);

        runtime.reset();
        while(runtime.seconds() < 1) {
            robot.leftMotor.setPower(-.5);
            robot.rightMotor.setPower(-.5);
        }

        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
        runtime.reset();
        while (runtime.seconds() < .75) {
            robot.leftMotor.setPower(1);
        }
        robot.leftMotor.setPower(0);
    }




    public String getColor(double red, double blue) {
        if(red >= blue) {
            return "red";
        } else {
            return "blue";
        }
    }
}
