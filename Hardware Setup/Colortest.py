
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
            block1pos = 1
            block2pos = 2
        else:
            block1pos = 2
            block2pos = 1
    elif len(blocks) == 3:
        blocklist = []
        for i in blocks:
            blocklist.append(i.y)
        count = 0
        for i in blocklist:
            if blocklist.index(i) != blocklist.index(max(blocklist)) and count != 1:
                block1 = blocks[blocklist.index(i)]
                count = 1
            elif blocklist.index(i) != blocklist.index(max(blocklist)) and count == 1:
                block2 = blocks[blocklist.index(i)]      
        block1_detected = 1
        block2_detected = 1
        block1_ID = block1.ID
        block2_ID = block2.ID
        if block1.x < block2.x:
            block1pos = 1
            block2pos = 2
        else:
            block1pos = 2
            block2pos = 1
    else:
        block1_ID = 0
        block2_ID = 0
        block1pos = 0
        block2pos = 0
    
    pr.update_channel('bloca', block1pos, block1_ID)
    pr.update_channel('blocb', block2pos, block2_ID)
    pr.process()





