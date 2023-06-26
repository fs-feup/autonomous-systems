import matplotlib.pyplot as plt

xTrackList = []
yTrackList = []
xMidList = []
yMidList = []

colorList = []
count = 0
with open('planning/files/map_test5.txt', 'r') as f:
    for line in f:   
        lineList = line.split(" ")
        lineList[2]=lineList[2].strip()
        if lineList[2] == "ol" or lineList[2] == "yellow": 
            # yellow and orange left -> left cones
            xTrackList.append(float(lineList[0]))
            yTrackList.append(float(lineList[1]))
            colorList.append("#000000") 
            # adds cone color in color list to the respective index
        elif lineList[2] == "or" or lineList[2] == "blue": 
            # blue and orange right -> right cones
            xTrackList.append(float(lineList[0]))
            yTrackList.append(float(lineList[1]))
            colorList.append("#0000ff")

print(len(xTrackList))

with open('planning/files/finalPath.txt', 'r') as f:
    for line in f:   
        lineList = line.split(" ")
        xMidList.append(float(lineList[0]))
        yMidList.append(float(lineList[1]))
        colorList.append("#aa0000")
            
plt.scatter(xTrackList+ xMidList, yTrackList + yMidList, s = 10, c = colorList)

plt.show()