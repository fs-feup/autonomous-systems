import json
  

def color_encoding(color):
    if color == "blue":
        return "b"
    elif color == "yellow":
        return "y"
    elif color == "orange_big":
        return "o"


# Opening JSON file
f = open('custom_track.json')
  
# returns JSON object as 
# a dictionary
data = json.load(f)

# Closing file
f.close()

f = open("src/map_mock.txt", "w")
for i in range (0, len(data['x'])):
    f.write(f"{data['x'][i]} {data['y'][i]} {color_encoding(data['color'][i])}\n")

  
# Iterating through the json
# list
# for i in data['emp_details']:
# for i in range (0, len(data['x'])):
#     print(f"{data['x'][i]} {data['y'][i]} {color_encoding(data['color'][i])}")


f.close()