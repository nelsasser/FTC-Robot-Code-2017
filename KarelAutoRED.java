package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.cam.Camera;

/**
 * Drives a little and launches ball at start
 */
@Autonomous(name="KarelAutoRED", group="Pushbot")
public class KarelAutoRED extends LinearOpMode {
    KarelHardware robot = new KarelHardware(); // use the class created to define karel's hardware

    ElapsedTime runtime = new ElapsedTime();

    private static final int TEAM_COLOR = 10; //for red team

    //Camera cam = new Camera();

    I2cDevice colorSensorR;
    I2cDevice colorSensorL;
    I2cDevice colorSensorF;

    I2cDeviceSynch colorReader1;
    I2cDeviceSynch colorReader2;
    I2cDeviceSynch colorReaderF;

    byte[] colorCache1;
    byte[] colorCache2;
    byte[] colorCacheF;

    boolean rightHasSeen = false;
    boolean leftHasSeen = false;

    ModernRoboticsI2cRangeSensor distanceSensor;

    //booleans for all of the lines
    boolean line1Found = false;
    boolean line2Found = false;
    boolean line3Found = false;
    boolean line4Found = false;

    //booleans for all of the beacons
    boolean b1Act = false;
    boolean b2Act = false;
    boolean b3Act = false;
    boolean b4Act = false;

    //boolean for when the ball is thrown
    boolean isThrown = false;

    @Override
    public void runOpMode() throws InterruptedException{
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        ///////////
        //SENSORS//
        ///////////
        distanceSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "distance"); //distance sensor
        boolean bLedOn = true; //led for color sensor

        colorSensorR = hardwareMap.i2cDevice.get("color1"); //color sensor 1
        colorSensorL = hardwareMap.i2cDevice.get("color2"); //color sensor 2
        colorSensorF = hardwareMap.i2cDevice.get("colorF");
        colorReader1 = new I2cDeviceSynchImpl(colorSensorR, I2cAddr.create8bit(0x3a), false);
        colorReader1.engage();
        colorReader2 = new I2cDeviceSynchImpl(colorSensorL, I2cAddr.create8bit(0x3c), false);
        colorReader2.engage();
        colorReaderF = new I2cDeviceSynchImpl(colorSensorF, I2cAddr.create8bit(0x3e), false);
        colorReaderF.engage();

        if(bLedOn) {
            colorReader1.write8(3, 0);
            colorReader2.write8(3, 0);
            colorReaderF.write8(3, 1); //we want to front led to always be passive
        } else {
            colorReader1.write8(3, 1);
            colorReader2.write8(3, 1);
            colorReaderF.write8(3, 0);
        }

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialized, waiting for start");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "am i here?");

        //do the ball
        shootBall();
        faceBeacon();

        //do the first beacon
        doFirstBeacon();


        //order of which things will take place
        // 1) go forward to set distance, launch ball
        // 2) go forward until closer set distance
        // 3) search for line 1
        // 4) activate beacon 1
        // 5) search for line 2
        // 6) activate beacon 2
        // 7) search for line 3
        // 8) activate beacon 3
        // 9) search for line 4
        // 10) activate beacon 4


    }

    public void doFirstBeacon() {
        while(!line1Found) {
            searchForLine1();
        }
        if(line1Found) {
            while (distanceSensor.getDistance(DistanceUnit.CM) > 5) {
                wiggle();
            }
            slam();
            runtime.reset();
            while(runtime.seconds() < 2) {
                //wait for beacon to regain itself
            }
            colorCacheF = colorReaderF.read(0x04, 1);
            int frontColor = colorCacheF[0] & 0xFF;
            if(frontColor != TEAM_COLOR) {
                slam();
            }
            runtime.reset();
            while(runtime.seconds() < 2) {
                //wait for beacon to regain itself
            }
            colorCacheF = colorReaderF.read(0x04, 1);
            frontColor = colorCacheF[0] & 0xFF;
            if(frontColor != TEAM_COLOR) {
                slam();
            }
        }
    }

    public void shootBall() {
        while(!isThrown) {
            if (!isThrown && distanceSensor.getDistance(DistanceUnit.CM) > 120) {
                go();
            } else if (!isThrown && distanceSensor.getDistance(DistanceUnit.CM) <= 120) {
                resetMotors();
                do180();
                throwBall();
                resetMotors();
                do180();
                isThrown = true;
                resetMotors();
            }
        }
    }

    public void faceBeacon() {
        runtime.reset();
        while (runtime.seconds() < 2) {
            robot.rightMotor.setPower(0);
            robot.leftMotor.setPower(-50);
        }
        resetMotors();
    }

    public void slam() {
        runtime.reset();
        while (runtime.seconds() < .5) {
            robot.rightMotor.setPower(-25);
            robot.leftMotor.setPower(-25);
        }
        runtime.reset();
        while (runtime.seconds() < .5) {
            robot.leftMotor.setPower(25);
            robot.rightMotor.setPower(25);
        }

        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
        runtime.reset();
    }

    //throws the stored ball
    public void throwBall() {
        runtime.reset();
        while (runtime.seconds() < 1) {
            robot.sweeper.setPower(-100);
        }
        robot.sweeper.setPower(0);
        runtime.reset();
    }

    //does a 180
    public void do180() {
        runtime.reset();
        while (runtime.seconds() < 3) {
            robot.leftMotor.setPower(-50);
            robot.rightMotor.setPower(50);
        }
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
        runtime.reset();
    }

    //goes forward
    public void go() {
        robot.leftMotor.setPower(-100);
        robot.rightMotor.setPower(-100);
    }

    //turns all motors off
    public void resetMotors() {
        robot.rightMotor.setPower(0);
        robot.leftMotor.setPower(0);
        robot.launcher.setPower(0);
        robot.sweeper.setPower(0);
    }

    //goes from after where the ball was thrown, to the line
    public void searchForLine1() {
        //this method will go forward for increasingly less distance, and it each stop it will comb for a line
        //the resume going forward;
        float timeToMove = 5;
        while(timeToMove > .15625) {
            runtime.reset();
            //move forward
            while(runtime.seconds() < timeToMove) {
                go();
            }
            resetMotors();
            if(sweep()) {
                return;
            } else {
                timeToMove/=2;
            }

        }
    }

    public boolean sweep() {
        runtime.reset();
        boolean anythingFound = false;
        //move right side
        while(runtime.seconds() < 1.5) {
            colorCache1 = colorReader1.read(0x04, 1);
            colorCache2 = colorReader2.read(0x04, 1);
            colorCacheF = colorReaderF.read(0x04, 1);

            robot.leftMotor.setPower(-35);
            robot.rightMotor.setTargetPosition(0);
            if(!leftHasSeen && (colorCache2[0]&0xFF) == 16) {
                leftHasSeen = true;
                rightHasSeen = false;
            } else if(!rightHasSeen && (colorCache1[0]&0xFF) == 16) {
                rightHasSeen = true;
                leftHasSeen = false;
            }
        }
        runtime.reset();
        //move left side
        while(runtime.seconds() < 3) {
            colorCache1 = colorReader1.read(0x04, 1);
            colorCache2 = colorReader2.read(0x04, 1);
            colorCacheF = colorReaderF.read(0x04, 1);

            robot.leftMotor.setPower(0);
            robot.rightMotor.setTargetPosition(-35);
            if(!leftHasSeen && (colorCache2[0]&0xFF) == 16) {
                leftHasSeen = true;
                rightHasSeen = false;
                return true;
            } else if(!rightHasSeen && (colorCache1[0]&0xFF) == 16) {
                rightHasSeen = true;
                leftHasSeen = false;
                return true;
            }
        }
        //reset so it is facing right direction again
        runtime.reset();
        while (runtime.seconds() < 1.5) {
            robot.leftMotor.setPower(-35);
            robot.rightMotor.setPower(0);
        }

        resetMotors();
        return false;
    }

    //wiggles back and forth along the line until it gets close to the beacon
    public void wiggle() {

        //turn on the motors
        if(leftHasSeen && !rightHasSeen) {
            runtime.reset();
            while(runtime.seconds() < .1) {
                robot.rightMotor.setPower(-75);
                robot.leftMotor.setPower(0);
            }
        } if(rightHasSeen && !leftHasSeen) {
            runtime.reset();
            while (runtime.seconds() < .1) {
                robot.leftMotor.setPower(-75);
                robot.rightMotor.setPower(0);
            }
        }

        //update which sensor has seen
        if(!leftHasSeen && (colorCache2[0]&0xFF) == 16) {
            leftHasSeen = true;
            rightHasSeen = false;
        } else if(!rightHasSeen && (colorCache1[0]&0xFF) == 16) {
            rightHasSeen = true;
            leftHasSeen = false;
        }
        runtime.reset();
    }

}
