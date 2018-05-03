## Project: 3D Motion Planning

![Quad Image](./misc/enroute.png)

---

## Udacity Flying Car Nanodegree  - 3D Motion Planning


# Implementation overview

## This project contains 2 implementations:
1. Base project with `TODO:` tasks completion. Code: [motion_planning.py](./motion_planning.py), [planning_utils.py](./planning_utils.py)
	Run command example:
    ```
    $>python motion_planning.py --goal -122.401289 37.796751 0.0
    ```
    Command line argument `--goal`is a goal global coordinates (longitude latitude altitude), i.e. '-122.401289 37.796751 0.0'

2. Graph-based path planning imlemetation by using approaches explained in the Lesson 7: Grids to Graphs Discretize. Code: [motion_planning_graph_imp.py](./motion_planning_graph_imp.py), [planning_utils.py](./planning_utils_graph_imp.py)
    Run command example:
    ```
    $>python motion_planning_graph_imp.py --goal -122.401289 37.796751 0.0
    ```


## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points 

#### 1. Explaination of the functionality provided in `motion_planning.py` and `planning_utils.py`
 [motion_planning.py](./motion_planning.py) is an executable script and it contains MotionPlanning class which is an extention of the Drone class provided by Udacity. This is an event driven implementation with message/events callback methods. Purpose of the MotionPlanning is to fly from the start location to the destination (goal) by finding a path in the environment with obstacles. Obstacles with global home coordinates are provided in the [colliders.csv](./colliders.csv) file. Sctript execution requires Udacity's Fly Simulator running. First it parses command line arguments, intializes MotionPlanning class instance and start the mission by making start() method call. After arming drone Script does path calculation and passes sequance of the local coordinates as trajectory target points for navigation. 
 [lanning_utils.py](/planning_utils.py) is a script with helper methods and drone Actions definition and validation. Main path calculation method is a_star (A*) 
 Utility methods:
- `create_grid` - Based on the obstacles data it crates a grid with value of a cell 1 if it's an obstacle and 0 otherwise. Method returns grid with offsets.
- `valid_actions` - method check and returns list of the valid in the provided grid moves from the location point.
- `a_star` - path calculation method for grid, start and goal points. Return sequense of the 2D waypoints.
-  `prune_path` - new method for culling path colinear points.
-  `collinearity_check` - checks points colinearity by calculating determinant of the points' vectors matrix
 
[colliders.csv](./colliders.csv) describes aproximated obstaclesis from the San Francisco downtown environment and here is the picture from above. 
![Map of SF](./misc/map.png)

### Path Planning Algorithm Implementation

#### 1. Bellow is a code snippet from the implemetation script which loads global home position from the first line of the csv file:
```Python
		
        # TODO: read lat0, lon0 from colliders into floating point values
        lat0 = None
        lon0 = None
        with open('colliders.csv') as f:
            #Read first line
            line=f.readline()
            #Split line after replacing latitude / longitude delimiter comma to whitespace for numpy 
            home_coord=np.loadtxt(StringIO(line.replace(', ', ' ')), delimiter=' ', usecols=(1, 3), unpack=True)
            lat0 = home_coord[0]
            lon0 = home_coord[1]
        print('Coordinates from the first line of the "colliders.csv" lat0 = {0},  lon0 ={1}'.format(lat0, lon0))
        
        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0.0)
```


#### 2. Setting a current local position
Base class Drone has properties for global positioning which supposes to be updated by the GPS
```Python
    
    # Global position in degrees (int)
    # Altitude is in meters
    self._longitude = 0
    self._latitude = 0
    self._altitude = 0
	...
    @property
    def global_position(self):
        return np.array([self._longitude, self._latitude, self._altitude])

```
Here is the implementation line from the code:
```Python
	global_pos = (self._longitude, self._latitude, self._altitude)
```

#### 3. Setting grid start position from local position
There is a utility method `global_to_local` provided by Udacity Drone package. Grid start position is relative to the offset.
```Python
	
    local_pos = global_to_local(global_pos, self.global_home)
    
    grid_start = (int(np.ceil(local_pos[0] - north_offset)), int(np.ceil(local_pos[1] - east_offset)))
```

#### 4. Setting grid goal position from geodetic coords
For ability to set arbitrary global position as a mission goal I have added public propert `goal_global_position` to the MotionPlanning class as well as a constructor parameter and command line aadditional argument '--goal'.  This argument is optional, default value ( -122.401289 37.796751 0.0) is coordinates of the Washington and Battery streets intersection  `plan_path` method uses property value by converting it to local grid poiint and passing as a destination for path computation.

#### 5. Adding diagonal motion 
`Action` enum in the [planning_utils.py](./planning_utils_graph_imp.py) defines all possible action considered in the A*. So I have extended this enum as followed:
```Python
  WEST = (0, -1, 1)
  EAST = (0, 1, 1)
  NORTH = (-1, 0, 1)
  SOUTH = (1, 0, 1)
  # Diagonal motions
  NORTHWEST = (-1, -1, np.sqrt(2))
  NORTHEAST = (-1, 1, np.sqrt(2))
  SOUTHWEST = (1, -1, np.sqrt(2))
  SOUTHEAST = (1, 1, np.sqrt(2))
```
Diagonal actions weight was set to sqrt(2) since direct ortogonal actions weight is 1. Also I had to modify `valid_actions` method for checking and using this additional moves realtive to the current position in the grid.

#### 6. Waypoints culling 
For waypoints culling I have implemented 2 methods `prune_path` and  `collinearity_check` in the [planning_utils.py](./planning_utils_graph_imp.py). After computing a path `prune_path`  call removes redundant colinear waypoints from the trajectory. Colinearity check uses determinant calculation.



### Execution
```
    $>python motion_planning.py --goal -122.401289 37.796751 0.0
```

# Extra Challenge: Real World Planning by using Graph

As described on the top I have implemented Graph-based path planning imlemetation by using approaches explained in the Lesson 7: Grids to Graphs. This implementation Script files: [motion_planning.py](./motion_planning.py), [planning_utils.py](./planning_utils.py). 
Path computation first creates Voronoi diagram by using obstacles middle points from the csv file, validates edge points against collision with obstacles  and creates navigation graph from this edges. 
A* traverses this graph for finding path from the start node to destination node which are the closest to the start and goal location ppoints accordingly.

Here is the video fragment captured from the Simulator screen:
[Video](./misc/FCND-Simulator 4_30_2018 7_34_33 PM.mp4)
