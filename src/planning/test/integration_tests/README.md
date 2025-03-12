# Testing Scenario Generation and Visualization

## Test Creation

The `scenario_generator.py` script is used to generate test scenarios for the system. Follow the steps below to create and save test files:

### Instructions

1. **Creating Cones, Starting Position and Final Box**:

   - Press `Y` and select points on the plot with the mouse to create **yellow cones**.
   - Press `B` and select points on the plot with the mouse to create **blue cones**.
   - Press `P` and select one point on the plot with the mouse to create the **starting point**.
   - Press `T` and select one point on the plot with the mouse to create the **target point**, indicating the direction where the car is pointing.
   - Press `F` to select two points (upper-left corner and lower-right corner) to define the **bounding box** for the final path point (ground truth for the test).

2. **Saving the File**:

   - Right-click with the mouse to save the file.
   - The file will be saved as `cones.txt` in the `integration_tests/tests/` folder. Rename this file to a new, descriptive name for clarity.

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

3. **Selecting a File to Visualize**:

   - When prompted in the terminal, enter the number corresponding to the debug file you wish to view.
   
   - **Understanding Debug File Structure**:
     - Debug files include additional information compared to standard test files:

       - **Vehicle Initial Position**:  
         Represented by a line starting with `V` (instead of P) followed by the x-coordinate, y-coordinate, and the vehicle's orientation (theta).  
         *Example:*  
         ```
         V 1.5 -0.2 1.38473
         ```
       - **Final Path Points**:  
         Represented by one or more lines starting with `P`, each containing the x and y coordinates of a point along the calculated path.  
         *Example:*  
         ```
         P 2.06738 1.81299
         P 2.08687 1.91947
         P 2.10737 2.02601
         P 2.12889 2.1326
         P 2.15142 2.23926
         P 2.17497 2.34598
         ```
        

4. The plot will display:

   - Cones (yellow and blue) - GROUND TRUTH, not calculated by the system
   - The final path - calculated by path planning algorithms
   - The initial position of the vehicle

5. To exit:

   - Close the plot window.
   - Enter `0` in the terminal to stop the script.

---

This workflow allows efficient creation, editing, and visualization of test scenarios, ensuring smooth integration testing.

