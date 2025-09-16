
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

while True:
    blocks = huskylens.get_blocks()

    if len(blocks) == 1:
        block1_detected = 1
        block2_detected = 0
        block1 = blocks[0]
        block1pos = 1
        block1_ID = block1.ID
        block2_ID = 0
        block2pos = 0

    elif len(blocks) == 2:
        block1_detected = 1
        block2_detected = 1
        block1 = blocks[0]
        block2 = blocks[1]
        block1_ID = block1.ID
        block2_ID = block2.ID
        if block1.x < block2.x:
            block1pos = 1  # left
            block2pos = 2  # right
        else:
            block1pos = 2
            block2pos = 1

    elif len(blocks) >= 3:
        # Pick the TWO blocks with the SMALLEST y (topmost two) — no imports
        top1 = None  # best (smallest y)
        top2 = None  # second best
        y1 = 10**9
        y2 = 10**9

        for b in blocks:
            y = b.y
            if y < y1:
                top2, y2 = top1, y1
                top1, y1 = b, y
            elif y < y2:
                top2, y2 = b, y

        # If we somehow didn't find two (shouldn't happen), fall back gracefully
        if top1 is None or top2 is None:
            block1_ID = 0
            block2_ID = 0
            block1pos = 0
            block2pos = 0
            block1_detected = 0
            block2_detected = 0
        else:
            block1_detected = 1
            block2_detected = 1

            # Order them by x so we can label left/right for the hub
            if top1.x < top2.x:
                left_block, right_block = top1, top2
            else:
                left_block, right_block = top2, top1

            block1 = left_block
            block2 = right_block

            block1_ID = block1.ID
            block2_ID = block2.ID
            block1pos = 1  # left
            block2pos = 2  # right

    else:
        block1_ID = 0
        block2_ID = 0
        block1pos = 0
        block2pos = 0
        block1_detected = 0
        block2_detected = 0
    
    pr.update_channel('bloca', block1pos, block1_ID)
    pr.update_channel('blocb', block2pos, block2_ID)
    pr.process()





