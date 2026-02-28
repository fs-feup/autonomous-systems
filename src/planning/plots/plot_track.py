# pylint: skip-file
# mypy: ignore-errors
import matplotlib.pyplot as plt

xTrackList = []
yTrackList = []
xMidList = []
yMidList = []

colorList = []
count = 0

print(len(xTrackList))

with open("../tracks/finalPath.txt", "r") as f:
    for line in f:
        lineList = line.split(" ")
        xMidList.append(float(lineList[0]))
        yMidList.append(float(lineList[1]))
        colorList.append("#00aa00")

plt.scatter(xTrackList + xMidList, yTrackList + yMidList, s=5, c=colorList)
plt.axis("equal")
plt.show()
