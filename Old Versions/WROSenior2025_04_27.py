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
defaultSpeed = 350
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

async def payloadsub():
    block1pos, block1id, b = await pr.call_async('bloca')
    print('Payload ID', block1id)
    if block1id == 1:
        await spin.run_angle(1000, 105, then=Stop.HOLD)
        await spin.run_angle(1000, -15, then=Stop.HOLD)
    elif block1id == 3:
        await spin.run_angle(1000, -105, then=Stop.HOLD)
        await spin.run_angle(1000, 15, then=Stop.HOLD)

    elif block1id == 4:
        await spin.run_angle(1000, 195, then=Stop.HOLD)
        await spin.run_angle(1000, -15, then=Stop.HOLD)

    else:
        pass
async def payloadbackup():
    await wait(400)
    await db.straight(-635, then=Stop.HOLD)

async def payload():
    db.use_gyro(True)
    db.settings(straight_speed=defaultSpeed)
    await db.straight(20, then=Stop.HOLD)
    await arm.run_angle(500, 510, then=Stop.HOLD)
    await db.straight(350, then=Stop.HOLD)
    await db.turn(90, then=Stop.HOLD)
    await db.straight(510, then=Stop.NONE)
    db.settings(straight_speed=50)
    await multitask(payloadsub(), db.straight(40, then=Stop.HOLD), arm.run_angle(500,-10))    
    db.settings(straight_speed=defaultSpeed)
    await multitask(arm.run_angle(-200, 500, then=Stop.HOLD),payloadbackup())
    await db.turn(-90, then=Stop.HOLD)

async def collectbolts():
    await db.straight(20, then=Stop.HOLD)
    await line(100, 190, 1)
    await db.straight(200, then=Stop.HOLD)
    await arm.run_angle(500, 200, then=Stop.HOLD)
    await db.straight(-350, then=Stop.HOLD)
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
    await arm.run_angle(300, 225, then=Stop.HOLD)
    await db.straight(-70, then=Stop.HOLD)
    await db.straight(20)
    await arm.run_angle(500, -225, then=Stop.HOLD)

async def alignbolts():
    await db.straight(-70)
    await db.turn(20)
    await db.straight(458)
    await db.turn(45)
    await db.straight(48)
    await arm.run_angle(500, -200, then=Stop.HOLD)
async def engagebolts():
    arm.control.limits(acceleration=1200)
    await db.straight(-20)
#    db.use_gyro(False)
    await arm.run_angle(500, 200, then=Stop.HOLD)
    await db.straight(-40)
    await arm.run_angle(500, -200, then=Stop.HOLD)
    await db.straight(-40)
    await arm.run_angle(500, 200, then=Stop.HOLD)
    await db.straight(-60)
    await arm.run_angle(500, 300, then=Stop.HOLD)

async def rotatebolts():    
    #Define initial square (BL, BR, FL, FR)
    #1: red
    #2: blue
    #3: green
    #4: yellow
    initial_square = ['3', '1', '2', '4']
    # Desired colors in the back row (BL, BR)
    block1pos, block1id, b = await pr.call_async('bloca')
    block2pos, block2id, b2 = await pr.call_async('blocb')
    if block1pos < block2pos: #if block1 is on left 
        desired_bl = str(block1id)
        desired_br = str(block2id)
    else: #else if it is on the right
        desired_bl = str(block2id)
        desired_br = str(block1id)
    print('Left Block ID:',desired_bl)
    print('Right Block ID:',desired_br)

    sequence, final_configuration = find_rotation_sequence(initial_square, desired_bl, desired_br)
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
#            await arm.run_angle(500, -300)
#            await db.straight(40)
#            await arm.run_angle(500, -200)
#            await db.straight(70)
            db.settings(straight_speed=120)
            await arm.run_angle(500,-80)
            await multitask(db.straight(120), arm.run_angle(500,-420))
            db.settings(straight_speed=defaultSpeed)
            await arm.run_angle(500,300)
            await db.straight(-120)
            await arm.run_angle(500,200)
            await spin.run_angle(500,185)
            await spin.run_angle(500,-5)
            await arm.run_angle(500, -300)
            await multitask(arm.run_angle(500, -200),db.straight(160))
            if count == len(sequence):
                isLastBack == 1
            else:
                await engagebolts()
    if isLastBack ==1:
        await db.straight(-20)
        await arm.run_angle(500, 250, then=Stop.HOLD)
        await db.straight(-50)
        db.use_gyro(True)
        await db.turn(90)
    else:
        db.settings(straight_speed=120)
        await arm.run_angle(500,-80)
        await multitask(db.straight(120), arm.run_angle(500,-420))
        db.settings(straight_speed=defaultSpeed)
        await arm.run_angle(500,250)
        await db.straight(-100)
        db.use_gyro(True)
        await db.turn(90)
    await wallsquare(-200,800)

async def yellowgatesub():
    await wait(300)
    await arm.run_angle(200, -250)
async def yellowgate():
    await multitask(db.straight(710), yellowgatesub())
    await db.turn(-90)
    await db.straight(300)
async def main():
    await payload()
    await collectbolts()
    await flags()
    await alignbolts()
    await engagebolts()
    await rotatebolts()
    await yellowgate()
#run_task(line(100,200,1))
print('Voltage:', hub.battery.voltage())
run_task(main())