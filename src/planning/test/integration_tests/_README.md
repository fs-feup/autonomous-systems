# Testing Scenario Generation and Visualization

## Test Creation

The `scenario_generator.py` script is used to generate test scenarios for the system. Follow the steps below to create and save test files:

### Instructions

1. **Creating Cones and Points**:

   - Press `Y` and select points on the plot with the mouse to create **yellow cones**.
   - Press `B` and select points on the plot with the mouse to create **blue cones**.
   - Press `P` and select one point on the plot with the mouse to create the **starting point**.
   - Press `T` and select one point on the plot with the mouse to create the **target point**, indicating the direction where the car is pointing.
   - Press `Z` to select two points (upper-left corner and lower-right corner) to define the **bounding box** for the final path point (ground truth for the test).

2. **Saving the File**:

   - Right-click with the mouse to save the file.
   - The file will be saved as `cones.txt` in the `integration_tests` folder. Rename this file to a new, descriptive name for clarity.

### Test File Structure

Ensure that your test file follows the correct structure:

- **Starting Point**:

  - There must be one line starting with `P` that includes the starting x and y coordinates and the starting angle.
  - Example:
    ```
    P 32.5 10.9 0.791415
    ```

- **Bounding Box**:

  - There must be one line starting with `F` that includes the minimum and maximum values of x and y, which define the bounding box.
  - Example:
    ```
    F -2 3 30 34.5
    ```

- **Cones**:

  - There must be at least one line starting with `C` that includes the x and y coordinates of the cones and their colors.
  - There is no limit to the number of cones.
  - Example:
    ```
    C 30.9 10.2 blue_cone
    C 25.4 11.5 yellow_cone
    ```

---

## Visualizing Results

1. Navigate to the `./results` folder.

2. Run the script `debug_plot.py` by typing:

   ```
   python debug_plot.py
   ```

3. When prompted, enter the number corresponding to the file you want to visualize.

4. The plot will display:

   - Cones (yellow and blue)
   - The final path

5. To exit:

   - Close the plot window.
   - Enter `0` in the terminal to stop the script.

---

This workflow allows efficient creation, editing, and visualization of test scenarios, ensuring smooth integration testing.

