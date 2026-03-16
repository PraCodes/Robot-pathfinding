# Robot Pathfinding with A* algorithm
A Python implementation of the A* search algorithm for robot path planning on occupancy grids derived from real maze images.

# How It Works
1. Loads the image and converts it to an occupancy grid
2. Detects obstacles, specifically yellow lego bricks in this case via HSV color masking and remaps them as blocked cells
3. Runs A* search from a defined start to goal position
4. Visualizes the original image, occupancy grid, and the discovered path

# Project Structure
```
robot-pathfinding/
│
├── robot_findpath.py   # Main script
├── maze2.png           # Input maze image
└── README.md
```

# Requirements
```bash
pip install numpy opencv-python matplotlib
```

# Usage

1. Place your maze image in the project folder
2. Update these variables in `robot_findpath.py`:
```python
image_path = "maze2.png"
start = (45, 45)    # (x, y) start coordinate
goal  = (45, 415)   # (x, y) goal coordinate
```

3. Run the script:
```bash
python robot_findpath.py
```

# Output

- Original Image — with start (green) and goal (red) marked
- Occupancy Grid — binary map of free vs. blocked cells
- Path Visualization — the A* path overlaid on the grid

# Notes

- White pixels = free space (navigable)
- Dark/grey pixels = obstacles
- Yellow objects are detected via HSV color masking and converted to grey before thresholding — this prevents bright yellow from being misread as free space
- Threshold for binarization is adjustable (`threshold=140`)

# Algorithm

Uses Manhattan distance as the heuristic for A* — suitable for 4-directional grid movement (up, down, left, right).
