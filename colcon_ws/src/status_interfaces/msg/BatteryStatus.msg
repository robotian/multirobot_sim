std_msgs/Header  header
float32 voltage          # Voltage in Volts (Mandatory)
float32 temperature      # Temperature in Degrees Celsius (If unmeasured NaN)
float32 current          # Negative when discharging (A)  (If unmeasured NaN)
float32 charge           # Current charge in Ah  (If unmeasured NaN)
float32 capacity         # Capacity in Ah (last full capacity)  (If unmeasured NaN)
float32 design_capacity  # Capacity in Ah (design capacity)  (If unmeasured NaN)
float32 percentage       # Charge percentage on 0 to 1 range  (If unmeasured NaN)
string remaining_time      # Time before battery discharges completely