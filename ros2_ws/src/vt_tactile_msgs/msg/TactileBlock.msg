# One decoded 26-byte A5 5A packet. These are ADC counts, not force/pressure.
# Ten values address (row, column_group + 8*k), k=0..9, in an 80x80 grid.
uint16 row
uint8 column_group
# Byte 4 has unknown firmware semantics. Preserve it; do not interpret it as CRC.
uint8 status_raw
uint16[10] adc
