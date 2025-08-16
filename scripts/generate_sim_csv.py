import os

csv_path = "./src/sim/data.csv"

NUM_STEPS = 1000

file = open(csv_path, "w")

file.write("Step,Steer,Accel,Brake\n")

steer = 0
accel = 0
brake = 0

for i in range(NUM_STEPS):
    # Steer
    if (i < 400): steer = 0
    elif (i < 500): steer = -((i - 399)/100)
    elif (i > 600 and i < 800): steer = ((i - 599)/100) - 1
    
    # Accel
    if (i < 100): accel = 0
    elif (i < 200): accel = (i - 100)/100
    elif (i < 300): accel = 1 - ( (i - 200)/200)

    file.write(f"{i},{steer},{accel},0\n")

file.close()
