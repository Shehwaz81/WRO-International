from pupremote_hub import PUPRemoteHub
from pybricks.parameters import Port, Direction, Axis, Color, Stop, Button
from pybricks.robotics import DriveBase
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.hubs import InventorHub
from pybricks.tools import multitask, run_task, wait

#Setup | Start Range 8 | Accept Range 15 | Block size threshold 20 
hub = InventorHub()
linesens = ColorSensor(Port.D)
arm = Motor(Port.C, Direction.CLOCKWISE)
spin = Motor(Port.F, Direction.COUNTERCLOCKWISE)
rmotor = Motor(Port.E, Direction.CLOCKWISE)
lmotor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
db = DriveBase(lmotor, rmotor, 62.4, 173)
pr = PUPRemoteHub(Port.A)
pr.add_channel('bloca','hhb') 
pr.add_channel('blocb','hhb')

#init variables
defaultSpeed = 475
defaultAcceleration = 900
flowAcceleration = 500

StartDist = 0
RealDist = 0
DriveError = 0
DrivePastError = 0
DriveDerivative = 0
DriveIntegral = 0
DrivePk = 0
DriveIk = 0
DriveDk = 0
Count_2 = 0
GyroStart = 0
DriveCorrection = 0
Pk = 0
Derivative = 0
Dk = 0
Error = 0
PastError = 0
Dist = 0
List = 0
Count = 0
ListNumber = 5
ListCount = 0
BlueCount = 0
ColorSum = 0
Dist2 = 0
time = 0
async def spinTarget(Target, Speed):
    await wait(0)
    # We do this because sometimes, the robot is weird and measures the angle 50
    # as -310, which are mathematically the same(if there are 360 degrees in a
    # circle, 50 degrees clockwise is the same as 310 degrees counterclockwise).
    # So, we check if the angle is less than -300, and if it is, we add 360
    # degrees to it so that (360)+(-310) = 50, and we rotate the correct amount
    if 0 > spin.angle():
        await spin.run_angle(Speed, Target - (360 + spin.angle()))
    else:
        await spin.run_angle(Speed, Target - spin.angle())
def flow():
    db.settings(straight_speed=375, turn_rate=90, straight_acceleration=500, turn_acceleration=500)
def normal():
    db.settings(straight_speed=defaultSpeed, turn_rate=150, straight_acceleration=defaultAcceleration, turn_acceleration=500)

async def block_release_right():
    await db.straight(-40)
    await arm.run_angle(500,250)
    await db.turn(-40)
    await db.turn(40)
    await arm.run_angle(500, -250)
    await db.straight(40)

async def block_release_left():
    await db.straight(-40)
    await arm.run_angle(500,250)
    await db.turn(40)
    await db.turn(-40)
    await arm.run_angle(500, -250)
    await db.straight(40)


async def start_timer():
    pass
# Rotations (using user's indexing: BL, BR, FL, FR)
# Rotations (using user's indexing: BL, BR, FL, FR)
# State layout: [BL, BR, FL, FR]

def rotate_square(square):
    # Rotate the whole 2x2 square clockwise
    return [square[2], square[0], square[3], square[1]]

def rotate_square_ccw(square):
    # Rotate the whole 2x2 square counterclockwise
    return [square[1], square[3], square[0], square[2]]

def rotate_square_180(square):
    # Rotate the whole 2x2 square by 180°
    return [square[3], square[2], square[1], square[0]]

def rotate_front_row(square):
    # Rotate (swap) the FRONT row: FL <-> FR
    # Input:  [BL, BR, FL, FR]  -> Output: [BL, BR, FR, FL]
    return [square[0], square[1], square[3], square[2]]

# BFS to find the shortest sequence of rotations to place BL/BR as desired
def find_rotation_sequence(start, goal_bl, goal_br):
    visited = set()
    queue = [(start, [])]  # (configuration, move_list)

    while queue:
        current, moves = queue.pop(0)  # FIFO
        key = tuple(current)
        if key in visited:
            continue
        visited.add(key)

        # Goal: back-left == goal_bl AND back-right == goal_br
        if current[0] == goal_bl and current[1] == goal_br:
            return moves, current

        # Explore moves (assumes these helpers exist and are synchronous)
        queue.append((rotate_square(current),     moves + ['cw']))   # 90° clockwise
        queue.append((rotate_square_ccw(current), moves + ['ccw']))  # 90° counterclockwise
        queue.append((rotate_square_180(current), moves + ['180']))  # 180°
        queue.append((rotate_front_row(current),  moves + ['fr']))   # front-row swap


async def callibrate_spin():
    while True:
        if hub.buttons.pressed() == {Button.LEFT}:
            spin.run_angle(100, 1, then=Stop.HOLD)
        if hub.buttons.pressed() == {Button.RIGHT}:
            spin.run_angle(100, -1, then=Stop.HOLD)
async def spin_square(times, direction):
    pass
async def spin_back():
    pass
async def spin_front():
    pass
async def wallsquare(speed,time):
    db.drive(speed, 0)
    await wait(time)
    db.stop()
async def line(Speed, Distance, Side):
    global Dist, DriveDk, DrivePk, DriveIk, DriveError, DriveIntegral, DriveDerivative, DriveCorrection, DrivePastError, RealDist
    await wait(0)
    db.use_gyro(False)

    black = 0
    white = 100
    target = (black+white)/2

    Dist = db.distance()
    DriveDk = 0.3
    DrivePk = 0.8
    DriveIk = 0.0002
    IntragalCap = 100
    while Distance >= db.distance() - Dist:
        await wait(0)
        DriveError = await linesens.reflection() - 50
        if DriveError == 0:
            DriveIntegral = 0
        else:
            DriveIntegral = DriveIntegral + DriveError

        if DriveIntegral > IntragalCap: DriveIntegral = IntragalCap
        elif DriveIntegral > IntragalCap: DriveIntegral = IntragalCap

        DriveDerivative = DriveError - DrivePastError
        DriveCorrection = Side * ((DrivePk * DriveError + DriveDk * DriveDerivative) + DriveIntegral * DriveIk)

        if abs(DriveError) > 30:
            DriveCorrection *= 1.1

        db.drive(Speed, DriveCorrection)
        DrivePastError = DriveError
        RealDist = db.distance() - Dist
    db.straight(0)
async def reflectedlightprint():
    while True:
        print(await linesens.reflection())
async def stupidlinesquare(speed):    
    db.drive(speed,0)
    print(await linesens.reflection())
    while not await linesens.reflection() <30:
        await wait(0)
    await db.straight(0)

async def payloadsub(blockid):
    global payloadreset
    print('Payload ID', blockid)
    if blockid == 1:
        payloadreset = -90
        await multitask(spin.run_angle(1200, 95, then=Stop.HOLD))
        await spin.run_angle(800, -5, then=Stop.HOLD)
    elif blockid == 3:
        payloadreset = 90
        await multitask(spin.run_angle(1200, -95, then=Stop.HOLD))
        await spin.run_angle(800, 5, then=Stop.HOLD)
    elif blockid == 4:
        payloadreset = -180
        await multitask(spin.run_angle(1200, 185, then=Stop.HOLD))
        await spin.run_angle(800, -5, then=Stop.HOLD)
    else:
        payloadreset = 0
async def spin_reset_delay(angle):
    await wait(200)
    print("reset angle:",angle)
    await spin.run_angle(2000, angle)
async def payload():
    db.use_gyro(True)
    db.settings(straight_speed=400, turn_rate=70, turn_acceleration=300)
    spin.control.limits(acceleration=2000)
    await multitask(spin.run_target(1000,0),db.straight(20, then=Stop.HOLD))
    await arm.run_angle(600, 520, then=Stop.HOLD)
    await db.straight(270, then=Stop.NONE)
    await stupidlinesquare(110)
    await db.straight(-0)
    await multitask(arm.run_angle(100,0),db.turn(90, then=Stop.HOLD))
    db.settings(straight_speed=400, turn_rate=140, straight_acceleration = 400)
    await db.straight(550, then=Stop.HOLD)
    block1pos, block1id, b = await pr.call_async('bloca')
    await multitask(payloadsub(block1id))
    normal()
  #  await payloadsub(block1id)
    arm.control.limits(acceleration=2800)
    await multitask(arm.run_angle(-600,425, then=Stop.HOLD),spin_reset_delay(payloadreset))
    arm.control.limits(acceleration=2800)
    normal()

    await multitask(arm.run_angle(-70,120, then=Stop.HOLD), db.straight(-620, then=Stop.HOLD, wait=True))
    arm.control.limits(acceleration=2800)    
    await db.turn(-87, then=Stop.NONE)
async def printangle():
    while True:
        print(db.angle())
        await wait(75)
async def collectbolts():
    await db.straight(25, then=Stop.NONE)
    await line(90, 165, -1)
    db.settings(straight_speed=260)
    db.drive(230,0)
    print(await linesens.reflection())
    while not await linesens.reflection() >=99:
        await wait(0)
    await db.turn(-2)
    await multitask(db.straight(140, then=Stop.HOLD))
    db.settings(straight_speed=defaultSpeed)
    await multitask(arm.run_angle(1000, 250, then=Stop.HOLD),db.turn(2))
    await db.straight(-325, then=Stop.HOLD)
async def flagdelay():
    await wait(400)
    await db.turn(25, then=Stop.HOLD)
async def flags():
    arm.control.limits(acceleration=1750)
    await db.turn(88)
    db.settings(straight_acceleration=700,turn_rate=60,straight_speed=200)
    await line(100,200,-1)
    print("angle reset")
    db.reset(angle=0)
    await wait(350)
    db.drive(125,0)
    print(await linesens.reflection())
    while not await linesens.reflection() >=99:
        await wait(0)
    await db.straight(0, then=Stop.HOLD)
    db.settings(straight_acceleration=1400,turn_rate=60,straight_speed=200)
    print("forward")
    await db.straight(170, then=Stop.HOLD)
    await wait(100)
    print("left turn 90")
    await db.turn(-90, then=Stop.HOLD)
    await wait(100)
    print("push first flag")
    await multitask(db.straight(-20, then=Stop.HOLD),arm.run_angle(320, 255, then=Stop.HOLD))
    db.settings(straight_acceleration=1400,turn_rate=60,straight_speed=210)

    #await wait(100)
    print("lift and turn")
    await multitask(arm.run_angle(240, -300, then=Stop.HOLD), flagdelay())
    #await wait(100)
    print("forward to next flag")
    await db.straight(120, then=Stop.HOLD)
    await arm.run_angle(240, 240, then=Stop.HOLD)
    db.settings(straight_speed=300)
    await multitask(arm.run_angle(400, -10, then=Stop.HOLD), db.straight(-80, then=Stop.HOLD))
    await db.straight(5)
    db.settings(straight_speed=defaultSpeed)
    normal()
    await arm.run_angle(300, -200, then=Stop.HOLD)



async def alignbolts():
    db.settings(turn_rate=80)
    print("left 90")
    db.settings(straight_acceleration=1400,turn_rate=50,straight_speed=200)
    await db.turn(-90-db.angle())
    await wait(200)
    print("back up")
    await db.straight(-70, then=Stop.HOLD)
    print("turn 45")
    await db.turn(45)
  #  await wait(150)
    print("straight")
    normal()
    db.settings(straight_speed=360)
    await db.straight(462)
#  await db.arc(-30, 2)
    print("offset angle:", db.angle())
    await arm.run_angle(300, -240, then=Stop.HOLD)
async def engagebolts():
    db.settings(straight_speed=450)
    arm.control.limits(acceleration=2500)
    await db.straight(-40)
    db.use_gyro(True)
    await arm.run_angle(400, 230, then=Stop.HOLD)
    await db.straight(-44)
    await arm.run_angle(300, -90, then=Stop.HOLD)
    await db.straight(-34)
    await arm.run_angle(300, 90, then=Stop.HOLD)
    await db.straight(-40)
    await arm.run_angle(280, 300, then=Stop.HOLD)
    await arm.run_angle(200,-3)
    await spin.run_angle(500,5)
    await spin.run_angle(500,-5)
    await multitask(db.straight(15),spin.run_angle(300,44))
async def scantimer():
    global block1id, block2id, block1pos, block2pos
    await wait(700)
    elapsed = 0
    interval = 100  # ms
    timeout = 1500  # ms
    while True:
        block1pos, block1id, b = await pr.call_async('bloca')
        block2pos, block2id, b2 = await pr.call_async('blocb')
        # if they are both equal, and if one block is equal to zero
        if (block1id == block2id) or (block1id == 0 or block2id == 0):
            await wait(interval)
            elapsed += interval
            if elapsed >= timeout:
                block1id = 3
                block2id = 1
                block1pos = 1
                block2pos = 2
                print('Scan failed')
                break
        else:
            print(block1id, block2id)
            print('Scanned successfully!')
            break

    print('Time to scan:', elapsed, 'ms')
async def wiggle():
    await db.straight(-5)
    await db.straight(5)
async def rotatebolts():    
    normal()
    global config
    #Define initial square (BL, BR, FL, FR)
    #1: red
    #2: blue
    #3: green
    #4: yellow
    initial_square = ['3', '1', '2', '4']
    # Desired colors in the back row (BL, BR)
    await scantimer()
    if block1pos < block2pos: #if block1 is on left 
        desired_bl = str(block1id)
        desired_br = str(block2id)
    else: #else if it is on the right
        desired_bl = str(block2id)
        desired_br = str(block1id)
    print('Left Block ID:',desired_bl)
    print('Right Block ID:',desired_br)
    sequence, final_configuration = find_rotation_sequence(initial_square, desired_bl, desired_br)
    config = [final_configuration[2], final_configuration[3]]
    print(type(config))
    print("Kids in the back: ", config)
    print('Sequence:',sequence)
    count = 0
    resetturn=0
    isLastBack = 0
   # await wait(30000)
    await spin.run_angle(500,-44)
    await wait(100)
    for move in sequence:
        count += 1
        if count == len(sequence):
            lastturn=45
            finalspeed=200
        else:
            lastturn=0
            finalspeed=0
        if move == "cw":
            await spin.run_angle(550-finalspeed,95+lastturn)
            await spin.run_angle(3000,-5)
            #await spin.run_angle(3000,15)
            resetturn = -90
        elif move == "ccw":
            await spin.run_angle(550-finalspeed,-95+lastturn)
            await spin.run_angle(3000,5)
            #await spin.run_angle(3000,-15)
            resetturn = 90
        elif move == "180":
            await spin.run_angle(550-finalspeed,-185+lastturn)
            await spin.run_angle(3000,5)
            #await spin.run_angle(3000,-14)
            resetturn = 180
        elif move == "fr":
            db.settings(straight_speed=120)
            arm.control.limits(acceleration=3000)
            await arm.run_angle(200,-120)
            await multitask(db.straight(-85), spin.run_angle(500, resetturn))
            await arm.run_angle(200,120)
            await arm.run_angle(200,-3)
            db.settings(straight_speed=275)
            arm.control.limits(acceleration=2000)
            await spin.run_angle(3000,180)
            await multitask(db.straight(30),arm.run_angle(160, -120))
            db.settings(straight_speed=120)
            await multitask(db.straight(95), spin.run_angle(3000,-180))
            await db.straight(-40)
            arm.control.limits(acceleration=2800)
            await arm.run_angle(280, 120)
            await arm.run_angle(200,-3)

    if 0==len(sequence):
        print("no sequence")
        await spin.run_angle(500,45)
        finaloffset=-45
    elif sequence[len(sequence)-1] == "ccw":
        finaloffset=-45
    elif sequence[len(sequence)-1] == "cw":
        finaloffset = -45
    elif sequence[len(sequence)-1] == "180":
        finaloffset = 135
    await arm.run_angle(500,-110)
    await multitask(db.turn(-25), spin.run_angle(500, finaloffset))
    await multitask(db.straight(-48),spin.run_angle(500,-20))
    await arm.run_angle(340,110)
async def yellowgatesub():
    await arm.run_angle(300, -500)
    await arm.run_angle(300, 250)
async def yellowgate():
    normal()
    await multitask(db.straight(-470,then=Stop.HOLD),spin.run_angle(500,110))
    normal()
    await db.turn(75)
    db.settings(straight_acceleration=600)
    await multitask(db.straight(895),yellowgatesub())
    normal()

async def redgate():
    await db.turn(-90)
    await db.straight(210)
    await arm.run_angle(400,235)
    db.settings(straight_speed=100, straight_acceleration=500)
    await db.straight(-124)
    db.settings(straight_speed=defaultSpeed, straight_acceleration=defaultAcceleration)
    await arm.run_angle(600,-235)
    await db.straight(-135, then=Stop.NONE)
    db.drive(-100,0)
    while not await linesens.reflection() <30:
        await wait(0)
    await db.arc(radius=50, angle=88, then=Stop.HOLD)

async def nosealign():
    arm.control.limits(acceleration=2000)
    await multitask(arm.run_angle(300,-200),line(190,300, -1), spin.run_target(500,0))
    db.settings(straight_speed=100, straight_acceleration=defaultAcceleration)
#    await line(190,300, -1) #delete later
    db.drive(200,0)
    print(await linesens.reflection())
    while not await linesens.reflection() >=99:
        await wait(0)
    await db.straight(-6)

async def nosedelivery():
    await arm.run_angle(290, 504, then=Stop.HOLD)
  #  await arm.run_angle(500,-10,then=Stop.COAST)
    db.settings(straight_speed=250, straight_acceleration=600)
    db.settings(straight_speed=defaultSpeed, straight_acceleration=600)
    db.straight(-30, then=Stop.NONE)
    await db.arc(radius=-170, angle=-90)
    await db.straight(-605)
    db.settings(straight_speed=defaultSpeed, straight_acceleration=defaultAcceleration)
    await rmotor.run_angle(240,230)
    await db.straight(207)
    await arm.run_angle(500,-275)
    db.settings(straight_speed=180)
    await db.straight(-97)
    await arm.run_angle(200,275)
    db.settings(straight_speed=defaultSpeed)
    await db.turn(-85)
    await arm.run_angle(550,-150)
    await multitask(db.straight(180, then=Stop.HOLD),arm.run_angle(550,-170))
    await db.straight(-40)
    await db.arc(radius=-70, angle=-126, then=Stop.NONE)
async def lastbolts():
    await db.straight(-70, then=Stop.NONE)
    await wallsquare(-200,800)
    await db.straight(75)
    await db.turn(90)
    await stupidlinesquare(100)
    await multitask(db.straight(100), arm.run_angle(400,-200))
    await db.straight(-190)
    await arm.run_angle(400,490)
    await db.straight(10)
    await wait(200)
    await multitask(scantimer(), db.straight(0))
    if block1pos > block2pos: #if block1 is on left 
        desired_bl = str(block1id)
        desired_br = str(block2id)
    else: #else if it is on the right
        desired_bl = str(block2id)
        desired_br = str(block1id)
    print("Desired Last Blocks:", desired_bl, desired_br)
  #  await db.turn(-2.5)
    await db.straight(-13)
    if desired_bl == config[1] and desired_br == config[0]:
        print("Rotation On Final")
        await spin.run_angle(2000,185)
        await spin.run_angle(500,-5)
        db.settings(straight_speed=130)
        arm.control.limits(acceleration=2000)
        await arm.run_angle(500,-80)
        await multitask(db.straight(190), arm.run_angle(500,-440), spin_reset_delay(-180))
        db.settings(straight_speed=400)
        await db.straight(-200)
    else:
        await multitask(arm.run_angle(500,-500), db.straight(-10))

async def surpriseprediction():
    db.settings(straight_speed=475, straight_acceleration=850)
    await db.turn(-90)
    await multitask(db.straight(730),arm.run_angle(300,200))
    db.drive(100,0)
    while not await linesens.reflection() <30:
        await wait(0)
    await db.arc(radius=72, angle=88, then=Stop.HOLD)
    await line(190,100, 1)
    await line(450,700, 1)

    
async def test():
    db.use_gyro(True)
    normal()
    await db.straight(-100)
    await db.turn(-90)
    await db.straight(450)
    await db.turn(90)
    await db.straight(50)
    await arm.run_angle(200,500)
    await db.straight(-100)
    await db.turn(-90)
    await db.straight(250)
    await db.turn(90)
    await db.straight(1300)
    await db.turn(90)
    await db.straight(340)

async def main():
    await start_timer()
    await payload()
    await collectbolts()
    await multitask(flags(),printangle(),race=True)
    await multitask(alignbolts(),printangle(),race=True)    
    await engagebolts()
    await rotatebolts()
    await yellowgate()
    await redgate()
    await nosealign()
    await nosedelivery()
    await lastbolts()
    await surpriseprediction()

#run_task(line(100,200,1))
print('Voltage:', hub.battery.voltage())
run_task(main())