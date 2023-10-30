
from random import random
from random import seed
from datetime import datetime
seed(datetime.now().timestamp())

f = open("map_250_rng.txt", 'w')

for i in range(266):
    if random() >= 0.5:
        color = "blue_cone"
    else:
        color = "yellow_cone"

    x = 100*random()
    y = 100*random()

    f.write(str(x) + " " + str(y) + " " + color + "\n")

f.close()



