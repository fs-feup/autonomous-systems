import numpy
import pandas as pd

data = pd.read_csv('hairpins_increasing_difficulty.csv')

f = open("hairpins.txt", 'w')
count = 0

for index, row in data.iterrows():
    color = row['tag']
    if color == "blue":
        strColor = "b"
    elif color == "yellow":
        strColor = "y"
    else:
        if count < 2: # MAY BE WRONG. add 2 orange rights and then the 2 orange left, 
            strColor = "or"
            count += 1
        else:
            strColor = "ol"
        

    f.write(str(row['x']) + " " + str(row['y']) + " " + strColor + "\n")

f.close()