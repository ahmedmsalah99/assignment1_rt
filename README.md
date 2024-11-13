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
  
1) When a boundry is violated it stops the vaiolator and then teleports it into the nearest safe lcoation
2) When distance is violated the turtles are stopped and then teleported into the nearest safe lcoation by an elegant way 
3) The distance and the violations are checked using a 100Hz rate function execution

* How to run ?

  Use
```bash
  rosrun assignment1_rt Distance
```
