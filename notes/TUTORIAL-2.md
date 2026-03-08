# Tutorial 2 Notes

**Tutorial 2:** https://www.youtube.com/watch?v=eeV9XHYXMlI

Not many notes are here as most of the tutorial will be covered in the simulation `README` and the lab instructions.

## Simulator Config
- Inside the config directory, `sim.yaml` has the config for the sim.
- Do not change:
	- Topics and namespaces
	- Transform related
	- LaserScan parameters
- Can change:
	- Map parameter (to use a different map)
	- Map agents: Multiple agents in the same map, each with their own topics
	- Ego and opponent starting pose
	- Whether `teleop` is enabled

## Topics
- `twist` measures velocity of the ego agent. `twist.twist.linear.x` is the longitudinal velocity in the car x-axis (velocity forwards). We don't really have `y` velocity.
- To drive the car we publish to the `/drive` topic.
