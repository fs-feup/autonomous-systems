import pandas as pd

data = pd.read_csv('skidpad.csv')

f = open("skidpad.txt", 'w')
count = 0

for index, row in data.iterrows():
    color = row['tag']
    if color == "blue":
        strColor = "b"
    elif color == "yellow":
        strColor = "y"
    else:
        if count < 2: # MAY BE WRONG. add 2 orange rights and then the 2 orange left, 
            strColor = "ol"
            count += 1
        else:
            strColor = "or"
        

    f.write(str(row['x']) + " " + str(row['y']) + " " + strColor + "\n")

f.close()