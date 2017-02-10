package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * DO NOT EDIT THIS.
 * THERE SHOULD BE NO REASON TO EVEN LOOK AT THIS
 * IF SOMETHING IS WRONG, I GUARANTEE NOTHING IS WRONG WITH THIS CLASS UNLESS YOU CHANGED IT!!!
 * CHANGE YOUR OTHER STUFF BECAUSE THAT IS PROBABLY WRONG
 */

/**
 * 1/17/17 Note: Uncommented launcher code b/c we are adding a launcher back
 */

public class KarelHardware {
    /* Public OpMode members. */
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;
    public DcMotor sweeper = null;
    public DcMotor launcher = null;

    /*
    public Servo leftClaw = null;
    public Servo  rightClaw = null;

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;*/

    /* local OpMode members. */
    HardwareMap hwMap =  null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public KarelHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotor = hwMap.dcMotor.get("left_drive");
        rightMotor = hwMap.dcMotor.get("right_drive");
        sweeper = hwMap.dcMotor.get("sweeper");
        launcher = hwMap.dcMotor.get("launcher");

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        launcher.setDirection(DcMotor.Direction.FORWARD);
        sweeper.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        sweeper.setPower(0);
        launcher.setPower(0);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        /*leftClaw = hwMap.servo.get("left_hand");
        rightClaw = hwMap.servo.get("right_hand");
        leftClaw.setPosition(MID_SERVO);
        rightClaw.setPosition(MID_SERVO);*/
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }

}
