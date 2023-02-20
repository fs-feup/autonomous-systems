import matplotlib.pyplot as plt

xLeftList = []
yLeftList = []
xRightList = []
yRightList = []
xMidList = []
yMidList = []

colorList = []
count = 0
with open('src\\files\\map_mock.txt', 'r') as f:
    for line in f:   
        lineList = line.split(" ")
        lineList[2]=lineList[2][0]
        if lineList[2] == "ol" or lineList[2] == "y": # yellow and orange left -> left cones
            xLeftList.append(float(lineList[0]))
            yLeftList.append(float(lineList[1]))
            colorList.append("#000000") # adds cone color in color list to the respective index, equal to the cone list(left + right + mid)
        elif lineList[2] == "or" or lineList[2] == "b": # blue and orange right -> right cones
            xRightList.append(float(lineList[0]))
            yRightList.append(float(lineList[1]))
            colorList.append("#0000ff")

print(len(xLeftList))
print(len(xRightList))
with open('src\\files\\finalPath.txt', 'r') as f:
    for line in f:   
        lineList = line.split(" ")
        xMidList.append(float(lineList[0]))
        yMidList.append(float(lineList[1]))
        colorList.append("#ff0000")

            
plt.scatter(xLeftList + xRightList + xMidList, yLeftList + yRightList + yMidList, s = 0.5, c = colorList)

plt.show()