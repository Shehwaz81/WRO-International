from machine import Pin, SoftI2C
from pupremote import PUPRemoteSensor
from pyhuskylens import HuskyLens, ALGORITHM_COLOR_RECOGNITION
import time

# --- Setup ---
pr = PUPRemoteSensor(power=True)
pr.add_channel('bloca','hhb')
pr.add_channel('blocb','hhb')
pr.process()

time.sleep(4)  # Huskylens boot delay
i2c = SoftI2C(scl=Pin(20), sda=Pin(19))
huskylens = HuskyLens(i2c)
print("Huskylens connected:", huskylens.knock())
huskylens.set_alg(ALGORITHM_COLOR_RECOGNITION)
huskylens.show_text("Hello LMS-ESP32 !")

last_block1_ID = 0
last_block2_ID = 0
frames_missing = 0


while True:
    try:
        blocks = huskylens.get_blocks()
        if blocks is None:
            blocks = []

        # Filter tiny/noisy blocks
        valid_blocks = [b for b in blocks if b.width > 5 and b.height > 5]

        block1_ID, block2_ID = 0, 0
        block1pos, block2pos = 0, 0

        if len(valid_blocks) == 1:
            block1 = valid_blocks[0]
            block1_ID = block1.ID
            block1pos = 1
            frames_missing = 0
        elif len(valid_blocks) == 2:
            block1, block2 = sorted(valid_blocks, key=lambda b: b.x)
            block1_ID, block2_ID = block1.ID, block2.ID
            block1pos, block2pos = 1, 2
            frames_missing = 0
        elif len(valid_blocks) >= 3:
            top2 = sorted(valid_blocks, key=lambda b: b.y)[:2]
            left_block, right_block = sorted(top2, key=lambda b: b.x)
            block1_ID, block2_ID = left_block.ID, right_block.ID
            block1pos, block2pos = 1, 2
            frames_missing = 0
        else:  # no valid blocks
            frames_missing += 1
            if frames_missing < 3:
                block1_ID, block2_ID = last_block1_ID, last_block2_ID
                block1pos, block2pos = 1 if last_block1_ID else 0, 2 if last_block2_ID else 0
            else:
                frames_missing = 0
                block1_ID, block2_ID = 0, 0
                block1pos, block2pos = 0, 0

        # Send to hub
        pr.update_channel('bloca', block1pos, block1_ID)
        pr.update_channel('blocb', block2pos, block2_ID)
        pr.process()

        # Save last block ids
        last_block1_ID, last_block2_ID = block1_ID, block2_ID

    except Exception as e:
        # Catch everything, log, and continue
        print("Error in loop:", e)

    time.sleep(0.05)
