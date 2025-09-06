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
def rotate_square(square):
    return [square[2], square[0], square[3], square[1]]
def rotate_square_ccw(square):
    return [square[1], square[3], square[0], square[2]]
def rotate_square_180(square):
    return [square[3], square[2], square[1], square[0]]
def rotate_back_row(square):
    return [square[1], square[0], square[2], square[3]]
# BFS function to find the shortest sequence of rotations
def find_rotation_sequence(start, goal_bl, goal_br):
    visited = []
    queue = [(start, [])]
    while queue:
        current, moves = queue.pop(0)
        key = tuple(current)
        
        if key in visited:
            continue
        visited.append(key)

        if current[0] == goal_bl and current[1] == goal_br:
            return moves, current

        queue.append((rotate_square(current), moves + ['cw']))  # 1: Rotate Square CW
        queue.append((rotate_square_ccw(current), moves + ['ccw']))  # 2: Rotate Square CCW
        queue.append((rotate_square_180(current), moves + ['180']))  # 3: Rotate Square 180
        queue.append((rotate_back_row(current), moves + ['br']))  # 4: Rotate Back Row

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
    Dist = db.distance()
    DriveDk = 0.3
    DrivePk = 1
    DriveIk = 0.0001
    while Distance >= db.distance() - Dist:
        await wait(0)
        DriveError = await linesens.reflection() - 50
        if DriveError == 0:
            DriveIntegral = 0
        else:
            DriveIntegral = DriveIntegral + DriveError
        DriveDerivative = DriveError - DrivePastError
        DriveCorrection = Side * ((DrivePk * DriveError + DriveDk * DriveDerivative) + DriveIntegral * DriveIk)
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
    global payloadbackup
    print('Payload ID', blockid)
    if blockid == 1:
        payloadbackup = -4
        await multitask(spin.run_angle(1200, 109, then=Stop.HOLD))
        await spin.run_angle(800, -19, then=Stop.HOLD)
    elif blockid == 3:
        payloadbackup = 5
        await multitask(spin.run_angle(1200, -101, then=Stop.HOLD))
        await spin.run_angle(800, 11, then=Stop.HOLD)
    elif blockid == 4:
        payloadbackup = 0
        await multitask(spin.run_angle(1200, 199, then=Stop.HOLD))
        await spin.run_angle(800, -19, then=Stop.HOLD)
    else:
        payloadbackup = 3
async def payload():
    db.use_gyro(True)
    db.settings(straight_speed=400, turn_rate=70, turn_acceleration=300)
    spin.control.limits(acceleration=2000)
    await spin.run_target(1000,0)
    await db.straight(20, then=Stop.HOLD)
    await arm.run_angle(600, 505, then=Stop.HOLD)
    await db.straight(270, then=Stop.NONE)
    await stupidlinesquare(110)
    await db.straight(-0)
    await multitask(arm.run_angle(100,0),db.turn(90, then=Stop.HOLD))
    db.settings(straight_speed=400, turn_rate=140, straight_acceleration = 400)
    await db.straight(500, then=Stop.HOLD)
    block1pos, block1id, b = await pr.call_async('bloca')
    await multitask(db.straight(50), payloadsub(block1id))
    normal()
  #  await payloadsub(block1id)
    arm.control.limits(acceleration=2800)
    await arm.run_angle(-600,395, then=Stop.HOLD)
    arm.control.limits(acceleration=2800)
    normal()

    await multitask(arm.run_angle(-100,120, then=Stop.HOLD), db.straight(-620, then=Stop.HOLD, wait=True))
    arm.control.limits(acceleration=2800)    
    await db.turn(-87, then=Stop.NONE)

async def collectbolts():
    await db.straight(25, then=Stop.NONE)
    await line(100, 165, -1)
    db.settings(straight_speed=230)
    db.drive(230,0)
    print(await linesens.reflection())
    while not await linesens.reflection() >=99:
        await wait(0)
    await db.straight(125, then=Stop.HOLD)
    db.settings(straight_speed=defaultSpeed)
    await arm.run_angle(1000, 250, then=Stop.HOLD)
    await db.straight(-310, then=Stop.HOLD)
async def flagdelay():
    await wait(400)
    await db.turn(25, then=Stop.HOLD)
async def flags():
    db.settings(straight_acceleration=defaultAcceleration)
    arm.control.limits(acceleration=1750)
    await db.turn(88)
    await line(150,170,-1)
    normal()
    db.reset(angle=0)
    await db.straight(220, then=Stop.HOLD)
    await db.turn(-90, then=Stop.HOLD)
    await multitask(db.straight(-20, then=Stop.HOLD),arm.run_angle(350, 250, then=Stop.HOLD))
    await multitask(arm.run_angle(500, -300, then=Stop.HOLD), flagdelay())
    await db.straight(120, then=Stop.HOLD)
    await arm.run_angle(300, 250, then=Stop.HOLD)
    db.settings(straight_speed=200)
    await multitask(arm.run_angle(400, -10, then=Stop.HOLD), db.straight(-75, then=Stop.HOLD))
    db.settings(straight_speed=defaultSpeed)
    await db.straight(20)
    await arm.run_angle(500, -200, then=Stop.HOLD)

async def alignbolts():
    normal()
    await db.straight(-80, then=Stop.HOLD)
    await db.turn(20)
    normal()
    await db.straight(483)
    print("offset angle:", db.angle())
    await db.turn(-db.angle())
    await stupidlinesquare(-125)
    await db.straight(70)
    await arm.run_angle(800, -240, then=Stop.HOLD)
async def engagebolts():
    db.settings(straight_speed=500)
    arm.control.limits(acceleration=2500)
    await db.straight(-40)
    db.use_gyro(True)
    await arm.run_angle(400, 230, then=Stop.HOLD)
    await db.straight(-44)
    await arm.run_angle(800, -90, then=Stop.HOLD)
    await db.straight(-34)
    await arm.run_angle(600, 90, then=Stop.HOLD)
    await db.straight(-40)
    await arm.run_angle(240, 295, then=Stop.HOLD)

async def scantimer():
    global block1id, block2id, block1pos, block2pos
    await wait(100)
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
    isLastBack = 0
    for move in sequence:
        count += 1
        if move == "cw":
            await spin.run_angle(3000,90)
            await spin.run_angle(3000,-15)
            await spin.run_angle(3000,15)
        elif move == "ccw":
            await spin.run_angle(3000,-90)
            await spin.run_angle(3000,15)
            await spin.run_angle(3000,-15)
        elif move == "180":
            await spin.run_angle(3000,-184)
            await spin.run_angle(3000,14)
            await spin.run_angle(3000,-14)
        elif move == "br":
            db.settings(straight_speed=120)
            arm.control.limits(acceleration=3000)
            await arm.run_angle(800,-120)
            await multitask(db.straight(120), arm.run_angle(640,-390))
            db.settings(straight_speed=275)
            arm.control.limits(acceleration=2000)
            await arm.run_angle(500,300)
            await db.straight(-120)
            await arm.run_angle(300,200)
            await spin.run_angle(3000,180)
            await spin.run_angle(3000,-15)
            await spin.run_angle(3000,15)
            await arm.run_angle(800, -300)
            arm.control.limits(acceleration=2800)
            await multitask(arm.run_angle(450, -200),db.straight(155))
            if count == len(sequence):
                isLastBack =  1
            else:
                await engagebolts()
    if isLastBack == 1:
        await db.straight(-42)
        await arm.run_angle(300, 250, then=Stop.HOLD)
        await db.straight(-113)
        db.use_gyro(True)
        await db.turn(90, then=Stop.HOLD)
    else:
        #await db.straight(150)
        #await stupidlinesquare(-125)
        db.settings(straight_speed=200)
        #await db.straight(70)    
        db.settings(straight_speed=120)
        arm.control.limits(acceleration=2800)
        await arm.run_angle(600,-140)
        arm.control.limits(acceleration=3000)
        await multitask(db.straight(119), arm.run_angle(350,-360))
        arm.control.limits(acceleration=1100)
        db.settings(straight_speed=350)
        await arm.run_angle(340,250)
        await db.straight(-109)
        db.use_gyro(True)
        await db.turn(90, then=Stop.HOLD)
    db.settings(straight_speed=defaultSpeed)
    await wallsquare(-200,900)
async def yellowgatesub():
    await wait(200)
    await arm.run_angle(300, -250)
    await arm.run_angle(300, 200)
async def yellowgate():
    await multitask(db.straight(700,then=Stop.HOLD), yellowgatesub())
    normal()
    await db.turn(-90)
    await db.straight(800)
    #await db.straight(135,then=Stop.HOLD)
    #await db.turn(13, then=Stop.HOLD)
   # await db.straight(60, then=Stop.HOLD)
  #  await db.turn(-23,then=Stop.HOLD)
 #   await db.straight(168, then=Stop.HOLD)
 #   await db.turn(11, then=Stop.HOLD)
    normal()
#    await db.straight(465)
async def redgate():
    await db.turn(-90)
    await db.straight(163)
    await arm.run_angle(400,285)
    db.settings(straight_speed=100, straight_acceleration=500)
    await db.straight(-124)
    db.settings(straight_speed=defaultSpeed, straight_acceleration=defaultAcceleration)
    await arm.run_angle(600,-285)
    await db.straight(-135, then=Stop.NONE)
    db.drive(-100,0)
    while not await linesens.reflection() <30:
        await wait(0)
    await db.arc(radius=50, angle=88, then=Stop.HOLD)

async def noses():
    arm.control.limits(acceleration=2000)
    await multitask(arm.run_angle(300,-200),line(190,280, -1), spin.run_target(500,0))
    db.settings(straight_speed=100, straight_acceleration=defaultAcceleration)
    db.drive(200,0)
    print(await linesens.reflection())
    while not await linesens.reflection() >=99:
        await wait(0)
    await db.straight(-15)
    await arm.run_angle(250, 520, then=Stop.COAST)
    db.settings(straight_speed=250, straight_acceleration=600)
    db.settings(straight_speed=defaultSpeed, straight_acceleration=600)
    db.straight(-30, then=Stop.NONE)
    await db.arc(radius=-180, angle=-90)
    await db.straight(-605)
    db.settings(straight_speed=defaultSpeed, straight_acceleration=defaultAcceleration)
    await rmotor.run_angle(140,230)
    await db.straight(215)
    await arm.run_angle(500,-250)
    db.settings(straight_speed=180)
    await db.straight(-90)
    await arm.run_angle(200,250)
    db.settings(straight_speed=defaultSpeed)
    await db.turn(-85)
    await arm.run_angle(550,-200)
    await multitask(arm.run_angle(550,-120), db.straight(150, then=Stop.HOLD))
    #await arm.run_angle(550,220)
    await db.straight(-50)
    await db.arc(radius=-70, angle=-122, then=Stop.NONE)
async def lastbolts():
    await db.straight(-70, then=Stop.NONE)
    await wallsquare(-200,800)
    await db.straight(85)
    await db.turn(89.5)
    await stupidlinesquare(100)
    await multitask(db.straight(100), arm.run_angle(400,-200))
    await db.straight(-200)
    await arm.run_angle(400,490)
    await db.straight(27)
    await wait(200)
    await multitask(scantimer(), db.straight(0))
    if block1pos < block2pos: #if block1 is on left 
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
        await multitask(db.straight(190), arm.run_angle(500,-440))
        db.settings(straight_speed=400)
        await db.straight(-200)
    else:
        await multitask(arm.run_angle(500,-500), db.straight(-10))
 #   print("Time before suprise: (s)", (time() / 1000))

async def surpriseprediction():
    db.settings(straight_speed=475, straight_acceleration=850)
    await db.turn(-90)
    await multitask(db.straight(730),arm.run_angle(300,200))
    await db.straight(-93)
    await db.turn(-90)
    await db.straight(90)
    db.settings(straight_speed=75, straight_acceleration=850)
    await db.straight(-100)
    await arm.run_angle(500,315)
    db.settings(straight_speed=475, straight_acceleration=850)
    await db.straight(-345)
    await db.turn(-90)
    await db.straight(-190)
#    print("Time after suprise: (s)", (time() / 1000))
    

async def test():
    db.settings(straight_speed=defaultSpeed)
    await db.straight(-100)
    await rmotor.run_angle(250,230)
async def main():
    await start_timer()
    await payload()
    await collectbolts()
    await flags()
    await alignbolts()
    await engagebolts()
    await rotatebolts()
    await yellowgate()
    await redgate()
    await noses()
    await lastbolts()
    #await surpriseprediction()
    

#run_task(line(100,200,1))
print('Voltage:', hub.battery.voltage())
run_task(main())