When sending downlinks though TTN, the bytes in complete message after : must be backwards compared to the docs (because it is LSB), 
    Ex. 0xABCD -> 0xCDAB, 0xFD 0x14 0xAE -> 0xAE 0x14 0xFD
This is gets confusing due to the message lacking any spacing between bytes/hex pairs

Checksums are calculated on the device side, do not include a checksum when sending HEX downlinks as it will be added automatically 

Load output off -> :8ABED0000
Load output user specified (default mode) -> :8ABED0005
Load output on override -> :8ABED0004
