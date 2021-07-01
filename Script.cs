const ulong INPUT_DELAY = 1200; // multiplier for how often user input is processed
const sbyte ACCELERATION = 2;   // max change in velocity between steps (m/s/step)
const sbyte MAX_SPEED = 10;     // mechanical limits of walker velocity
const sbyte MIN_SPEED = -6;     // ^

ulong time = 0;      // current 'time' of the animation system (stops when stationary)
sbyte mechSpeed = 0; // current theoretical velocity (m/s)

IMyShipController controller; // user control point
sbyte targetSpeed = 0;        // velocity to accelerate towards (m/s)
bool inputTaken = false;      // whether or not user input has been processed recently
ulong inputReset = 0;         // when to next accept user input for processing

Leg[] legs;

/**
* Initialize legs with animation data.
* NOTE : this method sets up the sentinel mech in the system, everything else can be applied to any robot performing repeating tasks at variable speeds.
*/
public Program(){
    string[] jointNames = new string[3];
    Pose[] poses;
    int startingPose;
    float[] pose = new float[3];

    controller = (IMyShipController) GridTerminalSystem.GetBlockWithName("Remote Control");
    legs = new Leg[6];

//Front Left Leg
    jointNames[0] = "Hinge FL1";
    jointNames[1] = "Hinge FL2";
    jointNames[2] = "Hinge FL3";

    poses = new Pose[12];

    // [0-7] in contact with ground (2.5m spacing), [8] lifting leg, [9 - 10] moving leg forward, [11] lower leg to ground
    pose = new float[3]; pose[0] = 33F;pose[1] =  -24F;pose[2] = 6F;poses[0] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = 30F;pose[1] =  -15F;pose[2] = -7F;poses[1] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = 20F;pose[1] =  5F;pose[2] = -21F;poses[2] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = 3F;pose[1] =  31F;pose[2] = -28F;poses[3] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = -7F;pose[1] =  44F;pose[2] = -33F;poses[4] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = -17F;pose[1] =  55F;pose[2] = -36F;poses[5] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = -27F;pose[1] =  64F;pose[2] = -38F;poses[6] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = -30F;pose[1] =  66F;pose[2] = -43F;poses[7] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = -30F;pose[1] =  90F;pose[2] = -90F;poses[8] = new Pose(pose, 2F);
    pose = new float[3]; pose[0] = 0F;pose[1] =  90F;pose[2] = -90F;poses[9] = new Pose(pose, 3F);
    pose = new float[3]; pose[0] = 35F;pose[1] =  -57F;pose[2] = 78F;poses[10] = new Pose(pose, 6.5F);
    pose = new float[3]; pose[0] = 35F;pose[1] =  -36F;pose[2] = 24F;poses[11] = new Pose(pose, 5.5F);

    startingPose = 0;

    legs[0] = new Leg(jointNames, poses, startingPose, this);

//Front Right Leg
    jointNames[0] = "Hinge FR1";
    jointNames[1] = "Hinge FR2";
    jointNames[2] = "Hinge FR3";

    startingPose = 7;

    legs[1] = new Leg(jointNames, poses, startingPose, this);

//Middle Left Leg
    jointNames[0] = "Rotor ML1";
    jointNames[1] = "Hinge ML2";
    jointNames[2] = "Hinge ML3";

    poses = new Pose[12];

    // [0-7] in contact with ground (2.5m spacing), [8] lifting leg, [9 - 10] moving leg forward, [11] lower leg to ground
    pose = new float[3]; pose[0] = -30F;pose[1] =  17F;pose[2] = -65F;poses[0] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = -35F;pose[1] =  22F;pose[2] = -72F;poses[1] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = -40F;pose[1] =  24F;pose[2] = -74F;poses[2] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = -46F;pose[1] =  26F;pose[2] = -75F;poses[3] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = -53F;pose[1] =  27F;pose[2] = -73F;poses[4] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = -60F;pose[1] =  25F;pose[2] = -66F;poses[5] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = -65F;pose[1] =  21F;pose[2] = -60F;poses[6] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = -70F;pose[1] =  18F;pose[2] = -55F;poses[7] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = -88F;pose[1] =  39F;pose[2] = -76F;poses[8] = new Pose(pose, 3.5F);
    pose = new float[3]; pose[0] = -75F;pose[1] =  75F;pose[2] = -90F;poses[9] = new Pose(pose, 3.5F);
    pose = new float[3]; pose[0] = -25F;pose[1] =  29F;pose[2] = -60F;poses[10] = new Pose(pose, 4F);
    pose = new float[3]; pose[0] = -25F;pose[1] =  13F;pose[2] = -60F;poses[11] = new Pose(pose, 6F);

    startingPose = 7;

    legs[2] = new Leg(jointNames, poses, startingPose, this);

//Middle Right Leg
    jointNames[0] = "Rotor MR1";
    jointNames[1] = "Hinge MR2";
    jointNames[2] = "Hinge MR3";

    poses = new Pose[12];

    // [0-7] in contact with ground (2.5m spacing), [8] lifting leg, [9 - 10] moving leg forward, [11] lower leg to ground
    pose = new float[3]; pose[0] = 30F;pose[1] =  17F;pose[2] = -65F;poses[0] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = 35F;pose[1] =  22F;pose[2] = -72F;poses[1] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = 40F;pose[1] =  24F;pose[2] = -74F;poses[2] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = 46F;pose[1] =  26F;pose[2] = -75F;poses[3] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = 53F;pose[1] =  27F;pose[2] = -73F;poses[4] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = 60F;pose[1] =  25F;pose[2] = -66F;poses[5] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = 65F;pose[1] =  21F;pose[2] = -60F;poses[6] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = 70F;pose[1] =  18F;pose[2] = -55F;poses[7] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = 88F;pose[1] =  39F;pose[2] = -76F;poses[8] = new Pose(pose, 3.5F);
    pose = new float[3]; pose[0] = 75F;pose[1] =  75F;pose[2] = -90F;poses[9] = new Pose(pose, 3.5F);
    pose = new float[3]; pose[0] = 25F;pose[1] =  29F;pose[2] = -60F;poses[10] = new Pose(pose, 4F);
    pose = new float[3]; pose[0] = 25F;pose[1] =  13F;pose[2] = -60F;poses[11] = new Pose(pose, 6F);

    startingPose = 0;

    legs[3] = new Leg(jointNames, poses, startingPose, this);

//Back Left Leg
    jointNames[0] = "Hinge BL1";
    jointNames[1] = "Hinge BL2";
    jointNames[2] = "Hinge BL3";

    poses = new Pose[12];

    // [0-7] in contact with ground (2.5m spacing), [8] lifting leg, [9 - 10] moving leg forward, [11] lower leg to ground
    pose = new float[3]; pose[0] = -85F;pose[1] =  47F;pose[2] = 48F;poses[0] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = -80F;pose[1] =  40F;pose[2] = 42F;poses[1] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = -75F;pose[1] =  31F;pose[2] = 33F;poses[2] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = -71F;pose[1] =  23F;pose[2] = 25F;poses[3] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = -67F;pose[1] =  10F;pose[2] = 11F;poses[4] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = -63F;pose[1] =  -3F;pose[2] = -3F;poses[5] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = -54F;pose[1] =  -25F;pose[2] = -20F;poses[6] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = -30F;pose[1] =  -90F;pose[2] = -65F;poses[7] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = -30F;pose[1] =  -75F;pose[2] = -65F;poses[8] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = -30F;pose[1] =  25F;pose[2] = 80F;poses[9] = new Pose(pose, 3.5F);
    pose = new float[3]; pose[0] = -50F;pose[1] =  10F;pose[2] = 90F;poses[10] = new Pose(pose, 6F);
    pose = new float[3]; pose[0] = -90F;pose[1] =  55F;pose[2] = 55F;poses[11] = new Pose(pose, 5F);

    startingPose = 0;

    legs[4] = new Leg(jointNames, poses, startingPose, this);

//Back Right Leg
    jointNames[0] = "Hinge BR1";
    jointNames[1] = "Hinge BR2";
    jointNames[2] = "Hinge BR3";

    poses = new Pose[12];

    // [0-7] in contact with ground (2.5m spacing), [8] lifting leg, [9 - 10] moving leg forward, [11] lower leg to ground
    pose = new float[3]; pose[0] = -85F;pose[1] =  47F;pose[2] = 48F;poses[0] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = -80F;pose[1] =  40F;pose[2] = 42F;poses[1] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = -75F;pose[1] =  31F;pose[2] = 33F;poses[2] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = -71F;pose[1] =  23F;pose[2] = 25F;poses[3] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = -67F;pose[1] =  10F;pose[2] = 11F;poses[4] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = -63F;pose[1] =  -3F;pose[2] = -3F;poses[5] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = -54F;pose[1] =  -25F;pose[2] = -20F;poses[6] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = -30F;pose[1] =  -90F;pose[2] = -65F;poses[7] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = -30F;pose[1] =  -75F;pose[2] = -65F;poses[8] = new Pose(pose, 2.5F);
    pose = new float[3]; pose[0] = -30F;pose[1] =  25F;pose[2] = 80F;poses[9] = new Pose(pose, 3.5F);
    pose = new float[3]; pose[0] = -50F;pose[1] =  10F;pose[2] = 90F;poses[10] = new Pose(pose, 6F);
    pose = new float[3]; pose[0] = -90F;pose[1] =  55F;pose[2] = 55F;poses[11] = new Pose(pose, 5F);

    startingPose = 7;

    legs[5] = new Leg(jointNames, poses, startingPose, this);
}

/**
* - parse user input
* - accelerate towards target speed if appropriate
* - continue walking animation if appropriate
* @param argument
* @param updateSource
*/
public void Main(string argument, UpdateType updateSource){
    userInput();

    if(mechSpeed == 0 || legs[0].hasLooped(time)){
        if(targetSpeed > mechSpeed){
            mechSpeed += ACCELERATION;
        } else if(targetSpeed < mechSpeed){
            mechSpeed -= ACCELERATION;
        }
    }

    Echo("target speed: " + targetSpeed);
    Echo("mech speed: " + mechSpeed);

    if(mechSpeed != 0){
        if(legs[0].hasLooped(time)){
            foreach(Leg leg in legs){
                leg.startLoop(time, mechSpeed);
            }
        } else{
            foreach(Leg leg in legs){
                leg.checkTime(time, mechSpeed);
            }
            time++;
        }
    } else{
        foreach(Leg leg in legs){
            leg.freeze();
        }
    }
}

/**
* Parses forward/backward input to increment/decrement target speed.
* When an input is read, ignores all input for a short time (to prevent over-sensitivity and double inputs)
*/
public void userInput(){
    if(!inputTaken && -controller.MoveIndicator.Z != 0){
        if(controller.MoveIndicator.Z < 0){
            if(targetSpeed < MAX_SPEED){
                targetSpeed += ACCELERATION;
            }
        } else{
            if(targetSpeed > MIN_SPEED){
                targetSpeed -= ACCELERATION;
            }
        }
        if(mechSpeed > 0){
            inputReset = time + (INPUT_DELAY * (ulong) mechSpeed);
        } else{
            inputReset = time + (INPUT_DELAY * (ulong) -mechSpeed);
        }

        inputTaken = true;
    } else if(inputReset == time){
        inputTaken = false;
    }
}

/**
* handles animation of a single leg.
*/
public class Leg{
    MyGridProgram console;
    Joint[] joints;
    Pose[] poses;
    int currentPose;
    int startingPose;
    ulong timeNext = 0; // time to go to next pose

    /**
    * Initialize leg with the given joints and animation data.
    * @param jointNames names of joints that are part of this leg
    * @param poses positions for this leg to move between when walking
    * @param startingPose index of first pose in walking animation
    * @param console reference to the programming block running this program
    */
    public Leg(string[] jointNames, Pose[] poses, int startingPose, MyGridProgram console){
        joints = new Joint[jointNames.Length];
        for(int i = 0; i < jointNames.Length; i++){
            if(jointNames[i].Contains("Rotor") || jointNames[i].Contains("Hinge")){
                joints[i] = new MotorJoint((IMyMotorStator) console.GridTerminalSystem.GetBlockWithName(jointNames[i]));
            } else if(jointNames[i].Contains("Piston")){
                joints[i] = new PistonJoint((IMyPistonBase) console.GridTerminalSystem.GetBlockWithName(jointNames[i]));
            } else{
                console.Echo("Error: could not parse joint: " + jointNames[i]);
            }
        }
        this.poses = poses;
        currentPose = startingPose;
        this.startingPose = startingPose;
        this.console = console;
    }

    /**
    * Checks if the leg is ready to start a new step.
    * @param time current time
    */
    public bool hasLooped(ulong time){
        if(currentPose == startingPose && timeNext == time){
            return true;
        } else{
            return false;
        }
    }

    /**
    * Move this leg to the second pose (increment from start if moving forward, decrement if reversing).
    * @param time current time
    * @param speed target walking speed
    */
    public void startLoop(ulong time, sbyte speed){
        currentPose = startingPose;
        if(speed > 0){
            timeNext = time + (ulong) (nextPose(speed) * 60F);
        } else{
            timeNext = time + (ulong) (prevPose(speed) * 60F);
        }
    }

    /**
    * Checks if it's time to go to the next pose.
    * - If so and returned to starting pose then stop
    * - If so and moving forward, increment pose
    * - if so and moving backward, decrement pose
    * @param time current time
    * @param speed target walking speed
    */
    public void checkTime(ulong time, sbyte speed){
        if(currentPose == startingPose && time == timeNext){
            freeze();
        } else if(time == timeNext && speed > 0){
            timeNext = time + (ulong) (nextPose(speed) * 60F);
        } else if(time == timeNext && speed < 0){
            timeNext = time + (ulong) (prevPose(speed) * 60F);
        }
    }

    /**
    * Move to next pose at a rate that would result in the given velocity when walking continuously (wraps).
    * @param speed
    * @return time until pose is reached
    */
    public float nextPose(sbyte speed){
        return goToPose(poses[incrementPose()], speed);
    }

    /**
    * Move to previous pose at a rate that would result in the given velocity when walking continuously (wraps).
    * @param speed
    * @return time until pose is reached
    */
    public float prevPose(sbyte speed){
        return goToPose(poses[decrementPose()], (sbyte) -speed);
    }

    /**
    * Stop all joints.
    */
    public void freeze(){
        foreach(Joint joint in joints){
            joint.freeze();
        }
    }

    /**
    * Move all joints towards their positions in the given pose at a rate that would result in the given speed when walking continuously.
    * @param pose
    * @param speed
    * @return time until pose is reached
    */
    private float goToPose(Pose pose, sbyte speed){
        for(int i = 0; i < joints.Length; i++){
            joints[i].goToPos(pose.positions[i], pose.baseTime / speed);
        }
        return pose.baseTime / speed;
    }

    /**
    * Increment current pose (wraps).
    * @return index of new pose
    */
    private int incrementPose(){
        if(currentPose == poses.Length - 1){
            currentPose = 0;
        } else{
            currentPose++;
        }
        return currentPose;
    }

    /**
    * Decrement current pose (wraps).
    * @return index of new pose
    */
    private int decrementPose(){
        if(currentPose == 0){
            currentPose = poses.Length - 1;
        } else{
            currentPose--;
        }
        return currentPose;
    }
}

/**
* Data structure for a single pose for a single leg with any number of joints.
*/
public class Pose{
    public float[] positions; // positions of each joint (degrees for rotors/hinges, metres for pistons)
    public float baseTime; // time to move to this pose for the mech to be moving at 1m/s

    /**
    * initialize with given parameters
    * @param positions positions of each joint (degrees for rotors/hinges, metres for pistons)
    * @param time time to move to this pose for the mech to be moving at 1m/s
    */
    public Pose(float[] positions, float baseTime){
        this.positions = positions;
        this.baseTime = baseTime;
    }
}

/**
* Handles a single piston joint.
*/
public class PistonJoint : Joint{
    IMyPistonBase piston;

    /**
    * Initialize with the given piston.
    */
    public PistonJoint(IMyPistonBase piston){
        this.piston = piston;
    }

    /**
    * Set piston speed to reach the given position at the given time.
    * @param position (m)
    * @param time (s)
    */
    public override void goToPos(float position, float time){
        float currentPosition = piston.CurrentPosition;
        piston.Velocity = (position - currentPosition) / time;
    }

    /**
    * Stop piston.
    */
    public override void freeze(){
        piston.Velocity = 0;
    }
}

/**
* Handles a single motor joint (rotor or hinge).
*/
public class MotorJoint : Joint{
    IMyMotorStator motor;

    /**
    * Initialize with given motor.
    */
    public MotorJoint(IMyMotorStator motor){
        this.motor = motor;
    }

    /**
    * Set motor speed to reach the given position in the given time.
    * @param position (degrees)
    * @param time (s)
    */
    public override void goToPos(float position, float time){
        float currentAngle = motor.Angle * 180F / (float) Math.PI;
        float distance = literalAngle(currentAngle, position);
        float dps = distance / time;
        float rpm = dps / 6F;
        motor.TargetVelocityRPM = rpm;
    }

    /**
    * Stop motor.
    */
    public override void freeze(){
        motor.TargetVelocityRPM = 0;
    }

    /**
    * Find the shortest angular distance between two angles.
    * @param start first angle
    * @param end second angle
    */
    private float literalAngle(float start, float end){
        float angle = end - start;

        if(angle >= 180F){
            angle -= 360F;
        } else if(angle < -180){
            angle += 360F;
        }

        return angle;
    }
}

/**
* Abstract class defining requirements for a joint.
*/
public abstract class Joint{
    public abstract void goToPos(float pose, float speed);
    public abstract void freeze();
}