from pupremote_hub import PUPRemoteHub
from pybricks.parameters import Port, Direction, Axis, Color, Stop, Button
from pybricks.robotics import DriveBase
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.hubs import InventorHub
from pybricks.tools import multitask, run_task, wait
#Setup
hub = InventorHub()
linesens = ColorSensor(Port.D)
arm = Motor(Port.C, Direction.CLOCKWISE)
spin = Motor(Port.F, Direction.COUNTERCLOCKWISE, gears=[12,36])
rmotor = Motor(Port.E, Direction.CLOCKWISE)
lmotor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
db = DriveBase(lmotor, rmotor, 62.4, 173)
pr = PUPRemoteHub(Port.A)
pr.add_channel('bloca','hhb') 
pr.add_channel('blocb','hhb')

#init variables
defaultSpeed = 400
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
async def stupidlinesquare():    
    db.drive(100,0)
    print(await linesens.reflection())
    while not await linesens.reflection() <30:
        await wait(0)
    db.stop()
async def payloadsub():
    block1pos, block1id, b = await pr.call_async('bloca')
    print('Payload ID', block1id)
    if block1id == 1:
        await spin.run_angle(1200, 108, then=Stop.HOLD)
        await spin.run_angle(800, -18, then=Stop.HOLD)
    elif block1id == 3:
        await spin.run_angle(1200, -108, then=Stop.HOLD)
        await spin.run_angle(800, 18, then=Stop.HOLD)

    elif block1id == 4:
        await spin.run_angle(1200, 198, then=Stop.HOLD)
        await spin.run_angle(800, -18, then=Stop.HOLD)

    else:
        pass

async def payload():
    db.use_gyro(True)
    db.settings(straight_speed=defaultSpeed, turn_rate=80)
    await db.straight(15, then=Stop.HOLD)
    await arm.run_angle(500, 515, then=Stop.HOLD)
    await db.straight(350, then=Stop.HOLD)
    await db.turn(90, then=Stop.HOLD)
    db.settings(straight_speed=defaultSpeed, turn_rate=175)
    await db.straight(510, then=Stop.NONE)
    db.settings(straight_speed=50)
    await multitask(payloadsub(), db.straight(40, then=Stop.HOLD), arm.run_angle(500,-10))    
    db.settings(straight_speed=defaultSpeed)
    arm.control.limits(acceleration=1800)
    await arm.run_angle(-550,400, then=Stop.HOLD)
    arm.control.limits(acceleration=2000)
    await multitask(arm.run_angle(-400,105, then=Stop.HOLD), db.straight(-635, then=Stop.HOLD))
    arm.control.limits(acceleration=2000)    
    await db.turn(-90, then=Stop.HOLD)

async def collectbolts():
    await db.straight(20, then=Stop.HOLD)
    await line(100, 190, 1)
    await db.straight(200, then=Stop.HOLD)
    await arm.run_angle(500, 200, then=Stop.HOLD)
    await db.straight(-330, then=Stop.HOLD)
async def flagdelay():
    await wait(400)
    await db.turn(25, then=Stop.HOLD)
async def flags():
    db.settings(straight_acceleration=680)
    arm.control.limits(acceleration=1750)
    await db.turn(90, then=Stop.HOLD)
    await line(150,200,-1)
    await db.straight(220, then=Stop.HOLD)
    await db.turn(-90, then=Stop.HOLD)
    await multitask(db.straight(-20, then=Stop.HOLD),arm.run_angle(500, 300, then=Stop.HOLD))
    await multitask(arm.run_angle(500, -300, then=Stop.HOLD), flagdelay())
    await db.straight(100, then=Stop.HOLD)
    await arm.run_angle(300, 228, then=Stop.HOLD)
    await multitask(arm.run_angle(400, -10, then=Stop.HOLD), db.straight(-60, then=Stop.HOLD))
    await db.straight(20)
    await arm.run_angle(500, -218, then=Stop.HOLD)

async def alignbolts():
    await db.straight(-80)
    await db.turn(20)
    await db.straight(462)
    await db.turn(45)
    await db.straight(56)
    await arm.run_angle(500, -200, then=Stop.HOLD)
async def engagebolts():
    arm.control.limits(acceleration=1000)
    await db.straight(-20)
    db.use_gyro(True)
    await arm.run_angle(500, 200, then=Stop.HOLD)
    await db.straight(-40)
    await arm.run_angle(500, -200, then=Stop.HOLD)
    await db.straight(-40)
    await arm.run_angle(500, 200, then=Stop.HOLD)
    await db.straight(-55)
    await arm.run_angle(300, 310, then=Stop.HOLD)

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
        if (block1id == block2id and (block1id == 0 or block2id == 0)) or (block1id == 0 or block2id == 0):
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
    await db.straight(-12)
async def rotatebolts():    
    global config
    #Define initial square (BL, BR, FL, FR)
    #1: red
    #2: blue
    #3: green
    #4: yellow
    initial_square = ['3', '1', '2', '4']
    # Desired colors in the back row (BL, BR)

    await multitask(wiggle(),scantimer())
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
            await spin.run_angle(500,95)
            await spin.run_angle(500,-5)
        elif move == "ccw":
            await spin.run_angle(500,-95)
            await spin.run_angle(500,5)
        elif move == "180":
            await spin.run_angle(500,-185)
            await spin.run_angle(500,55)
        elif move == "br":
            db.settings(straight_speed=120)
            arm.control.limits(acceleration=2000)
            await arm.run_angle(500,-80)
            await multitask(db.straight(130), arm.run_angle(500,-430))
            db.settings(straight_speed=275)
            arm.control.limits(acceleration=1100)
            await arm.run_angle(500,300)
            await db.straight(-130)
            await arm.run_angle(300,210)
            await spin.run_angle(500,185)
            await spin.run_angle(500,-5)
            await arm.run_angle(500, -300)
            arm.control.limits(acceleration=1100)
            await multitask(arm.run_angle(500, -210),db.straight(155))
            if count == len(sequence):
                isLastBack =  1
            else:
                await engagebolts()
    if isLastBack == 1:
        await db.straight(-20)
        await arm.run_angle(300, 250, then=Stop.HOLD)
        await db.straight(-115)
        db.use_gyro(True)
        await db.turn(90)
    else:
        db.settings(straight_speed=120)
        arm.control.limits(acceleration=1100)
        await arm.run_angle(500,-80)
        arm.control.limits(acceleration=2000)
        await multitask(db.straight(120), arm.run_angle(500,-420))
        arm.control.limits(acceleration=1100)
        db.settings(straight_speed=275)
        await arm.run_angle(500,250)
        await db.straight(-100)
        db.use_gyro(True)
        await db.turn(90)
    db.settings(straight_speed=defaultSpeed)
    await wallsquare(-200,900)
async def yellowgatesub():
    await wait(300)
    await arm.run_angle(200, -250)
    await arm.run_angle(250, 200)
async def yellowgate():
    await multitask(db.straight(730), yellowgatesub())
    await db.turn(-90)
    await db.straight(120)
    await db.turn(15)
    await db.straight(55)
    await db.turn(-25)
    await db.straight(155)
    await db.turn(10)
    await db.straight(480)
async def redgate():
    await db.turn(-90)
    await db.straight(150)
    await arm.run_angle(400,240)
    db.settings(straight_speed=50)
    await db.straight(-95)
    db.settings(straight_speed=defaultSpeed)
    await arm.run_angle(400,-240)
    await db.straight(-199)
    await db.turn(90)
async def noses():
    arm.control.limits(acceleration=2000)
    await line(220,300, -1)
    await db.straight(115)
    await arm.run_angle(350, 303)
    #await arm.run_angle(500,-2, then=Stop.COAST)
    await db.straight(-50)
    await db.turn(30)
    await db.straight(-100)
    await db.turn(30)
    await db.straight(-50)
    await db.turn(35)
    await db.straight(-620)
#    await db.straight(20)
    await db.turn(-45)
    await db.straight(250)
    await db.turn(4)
    await arm.run_angle(500,-110)
    await db.turn(-4)
    await db.straight(-105)
    await arm.run_angle(300,120)
    #await arm.run_angle(500,-5, then=Stop.COAST)
    await db.straight(-10)
    await db.turn(-90)
    await multitask(db.straight(160), arm.run_angle(400,-475))
    await arm.run_angle(450,200)
async def lastbolts():
    await db.straight(-100)
    await db.turn(45)
    await db.straight(-120)
    await db.turn(90)
    await db.straight(-160)
    await wallsquare(-200,800)
    await db.straight(85)
    await db.turn(90)
    await stupidlinesquare()
    await multitask(db.straight(110), arm.run_angle(400,-200))
    await db.straight(-200)
    await arm.run_angle(400,500)
    global block1id, block2id, block1pos, block2pos
    block1pos, block1id, b = await pr.call_async('bloca')
    block2pos, block2id, b2 = await pr.call_async('blocb')
    if block1pos < block2pos: #if block1 is on left 
        desired_bl = str(block1id)
        desired_br = str(block2id)
    else: #else if it is on the right
        desired_bl = str(block2id)
        desired_br = str(block1id)
    if desired_bl == config[1] and desired_br == config[0]:
        print("Rotation On Final")
        await spin.run_angle(500,185)
        await spin.run_angle(500,-5)
        db.settings(straight_speed=130)
        arm.control.limits(acceleration=2000)
        await arm.run_angle(500,-80)
        await multitask(db.straight(155), arm.run_angle(500,-430))
    else:
        pass


async def main():
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

#run_task(line(100,200,1))
print('Voltage:', hub.battery.voltage())
run_task(main())