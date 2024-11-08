# assignment1_rt

### Setup

While in the root workspace dir execute

```bash
  catkin_make
  source devel/setup.sh
```

Before running the nodes make sure to run turtle sim using 

```bash
  rosrun turtlesim turtlesim_node
```

#### UI

* Behaviour

1) Spawns a turtle with the name "turtle2" at x=10.5, y=5.5, with theta=0
2) Starts a loop to get user input for commanding the turtles
3) Use input checking is there. The code accepts "turtle1" and "turtle2" only as names of the turtle
4) For the velocity commands code accepts doubles only
5) The command continue for 1 second and then stops

* How to run ?

  Use
```bash
  rosrun assignment1_rt UI
```


#### Distance

* Behaviour
  
1) Assumes starting positions for the turtles that don't violate the condtions
2) When a boundry is violated it teleports the vaiolator turtle into a safe pose and stops it
3) When distance is violated turtle 2 is teleported into a safe space by teleporting up or down based on its relative pose with turtle1
4) Also, both turtles stops if distance condition violated (1.0 threshold)
5) The distance and the violations are checked using a 100/s rate function execution

* How to run ?

  Use
```bash
  rosrun assignment1_rt Distance
```
