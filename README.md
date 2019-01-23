# secretbandname
Bipedal Robot lab project for ECE4560 | Fall 2017

## Data
#### Link Lengths <Spacing between the equations affects formatting>

![L0](https://latex.codecogs.com/gif.latex?L_0=104.775mm)

![L1](https://latex.codecogs.com/gif.latex?L_1=107.95mm)

![L2](https://latex.codecogs.com/gif.latex?L_2=107.95mm)

![L3](https://latex.codecogs.com/gif.latex?L_3=15.875mm)

![L4](https://latex.codecogs.com/gif.latex?L_4=38.1mm)

#### Motors
![M](https://latex.codecogs.com/gif.latex?m_{motor}=53.5g)

![XM](https://latex.codecogs.com/gif.latex?x_{motor}=32mm)

![YM](https://latex.codecogs.com/gif.latex?y_{motor}=50mm)

![ZM](https://latex.codecogs.com/gif.latex?z_{motor}=40mm)

### Angle Measurement
Angles relative to each frame are measured from the **negative y-axis**. Exactly like this [pendulum's angle](https://upload.wikimedia.org/wikipedia/commons/b/b2/Simple_gravity_pendulum.svg).

### Internal Data Management
[Rough Sketch of Setup](refs/orientation.jpg)

* Link lengths are stored as an Nx1 vector
	* Both biped legs are identical in geometry
	* Link indices follow this [diagram](refs/frames.png)
	* Lengths are in **millimeters**
	
* Joint angles are stored as an Nx2 matrix
	* The biped legs can be configured independently
	* 1st column corresponds with the left leg
	* 2nd column corresponds with the right leg
	* Angles are in **radians**

* Link COM and Motor COM data are input separately
	* Link and Motor COMs don't correspond intuitively
	* Need to perfom link-specific motor centroid translations
	* Handling calculations internally removes them from the input
	* Link COM data included for forward-compatibility
	
* One full gait trajectory is stored in [gaitData.mat](labs/gaitData.mat)
	* Moving right foot data is stored in [swingRight.mat](labs/swingRight.mat)
	* Recentering right foot data is stored in [doubleRL.mat](labs/doubleRL.mat)
	* The left foot trajectory is reated by mirroring right foot trajectories


### Stability
See this [document](refs/supportcurve.md).

### Zero-Configuration
All joints in a vertical line, with their y-axes parallel to that line.


## Notes
* L<sub>3</sub> & L<sub>4</sub> extend along the Foot Frame x-axis
* ```update_linkframes``` is called every time the joint angles change
* L<sub>0</sub> was calulated by doubling the plastic connector's length
* Link length measurements **include** motors' contributions
* For lab scripts, got to [labs/](labs/)
* Be sure to clone the Optragen submodule for trajectory generation

## Authors
* **Madeleyne Vaca**
* **Marion Anderson**

## References
* [Biped Class Wiki](http://pvela.gatech.edu/classes/doku.php?id=ece4560:biped:adventures)

## Special Thanks
* **Alex Chang** - ECE4560 TA
* **Patricio Vela** - ECE4560 Professor

## License
It should be noted that not all the work here is mine. In particular, this project was a class assignment, and I had no involvement whatsoever in the development of Optragen. 

However, the MATLAB code in this repository, beyond skeleton code for the SE2 and Biped classes, was developed by Madeleyne and me.

 We have no issue with that code being used for inspiration, knowledge gap-filling, or utility, but it is the result of academic coursework. Deliberate duplication with intent to substitue understanding in an academic setting would constitute plagiarism.
