## 1. Introduction

This Repository Generates a Catmull–Rom Spline Path with the given points. 

## 2. Clone the repository.

```
mkdir my_ws
cd my_ws
mkdir src
cd src

```

## 3. Build the project.

```
cd ..
colcon build
```

## 4. Launch the rviz and the node

```
rviz2
ros2 run spline_generator catmull_rom_node
```

## 5. Path generated

![path.png](images/path.png)

## 

![path_2.png](images/path_2.png)

## 6. Approach

- - The resolution is 0.1 so all the path generated through the formula is discretized in the loop considering points at distance of 0.1 and thereby generating the path.