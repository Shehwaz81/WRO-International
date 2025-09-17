
from machine import Pin
from machine import SoftI2C
from pupremote import PUPRemoteSensor
from pyhuskylens import HuskyLens, ALGORITHM_COLOR_RECOGNITION
import time


# Set up comms with SPIKE hub
pr = PUPRemoteSensor(power=True)
pr.add_channel('bloca','hhb') # Pass two 'h'alf ints: x coordinate of line head, and of line tail.
pr.add_channel('blocb','hhb') # Pass two 'h'alf ints: x coordinate of line head, and of line tail.
pr.process() # Connect to hub

# Set up Huskylens
# Ensure Huskylens is in i2c mode via General Settings > Protocol Type
time.sleep(4) # Wait for the Huskylens to boot
i2c = SoftI2C(scl=Pin(20), sda=Pin(19))
huskylens = HuskyLens(i2c)
print("Huskylens connected is", huskylens.knock())
huskylens.set_alg(ALGORITHM_COLOR_RECOGNITION)
huskylens.show_text("Hello LMS-ESP32 !")    

last_block1_ID, last_block2_ID = 0, 0
frames_missing = 0

while True:
    blocks = huskylens.get_blocks()
    valid_blocks = []
    for b in blocks:
        if b.width > 5 and b.height > 5:
            valid_blocks.append(b)
    
    if len(valid_blocks) == 1:
        block1, = valid_blocks
        block1_detected = 1
        block2_detected = 0
        block1pos = 1
        block1_ID = block1.ID
        block2_ID = 0
        block2pos = 0
        frames_missing = 0;

    elif len(valid_blocks) == 2:
        block1_detected = 1
        block2_detected = 1
        block1, block2 = valid_blocks
        block1_ID = block1.ID
        block2_ID = block2.ID
        frames_missing = 0;
        if block1.x < block2.x:
            block1pos = 1  # left
            block2pos = 2  # right
        else:
            block1pos = 2
            block2pos = 1

    elif len(valid_blocks) >= 3:
        top2 = sorted(valid_blocks, key=lambda b: b.y)[:2]
    
        # Order them left/right by x
        left_block, right_block = sorted(top2, key=lambda b: b.x)
    
        block1 = left_block
        block2 = right_block
    
        block1_ID = block1.ID
        block2_ID = block2.ID
        block1pos = 1
        block2pos = 2
        block1_detected = 1
        block2_detected = 1
        frames_missing = 0;
    else:
        frames_missing += 1
        if frames_missing < 3:
            block1_ID, block2_ID = last_block1_ID, last_block2_ID
        else:
            frames_missing = 0
            block1pos, block2pos = 0, 0
            block1_ID, block2_ID = 0, 0
    
    pr.update_channel('bloca', block1pos, block1_ID)
    pr.update_channel('blocb', block2pos, block2_ID)
    pr.process()

    last_block1_ID, last_block2_ID = block1_ID, block2_ID
    time.sleep(0.05)





