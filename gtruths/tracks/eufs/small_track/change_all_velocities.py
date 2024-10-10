import csv

file_path = "/home/ws/gtruths/tracks/eufs/small_track/small_track_gtruth.csv"
desired_velocity = 2

with open(file_path, mode="r", newline="") as infile:
    reader = csv.reader(infile)
    rows = list(reader)

with open(file_path, mode="w", newline="") as outfile:
    writer = csv.writer(outfile)

    # Write the header (first line) unchanged
    writer.writerow(rows[0])

    # Modify and write the remaining rows
    for row in rows[1:]:
        if len(row) == 3:
            row[2] = str(desired_velocity)
        writer.writerow(row)
