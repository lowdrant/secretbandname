# Support Curve
A curve that bounds the planar area over which a rigid body's center of mass is statically stable when acted upon only by gravity.

When only one foot is on the ground, the biped's support curve is any point directly above that foot. When two feet are on the ground, the support curve runs from the back edge of the rear foot to the front edge of the front foot.


## Force Analysis
* Biped is "stable" IFF ![st](https://latex.codecogs.com/png.latex?\Sigma\tau=0)
	* Stable here roughly means "not falling over"
	* The biped could still tip over if it is already moving
	
* 2 to 3 forces acting on biped: 
	* F<sub>g</sub> = mg
	* F<sub>n</sub> = F<sub>n<sub>1</sub></sub> + F<sub>n<sub>2</sub></sub>= -F<sub>g</sub>
	
* Three points of application:
	* F<sub>g</sub> acts through the COM
	* F<sub>n</sub> can act through either, or both, feet
	
	
	
	
## Stability
* F<sub>n</sub> has a point of application in each foot
	* Point of application varies with COM x-coordinate
* Its most extreme point of application is at the edge of the footprint


### Single Foot Case
* F<sub>n</sub> = -F<sub>g</sub>
* F<sub>n</sub>  will share a line of action with F<sub>g</sub> whenever possible
	* This arrangement produces no torque
* If F<sub>n</sub> cannot share a line of action with F<sub>g</sub>, they create a couple
	* Generates a net torque along the foot frame x-axis
		* Roughly equal to | F<sub>g</sub> | * | x<sub>com</sub> -  x<sub>foot</sub> |
	* Occurs when the COM is beyond  footprint edge
	
	
### Two Foot Case
* F<sub>n<sub>1</sub></sub> + F<sub>n<sub>2</sub></sub> = -F<sub>g</sub>
* F<sub>g</sub> between the two feet
	* F<sub>g</sub> will create a moment about the rear foot (foot 1)
	* F<sub>n<sub>2</sub></sub> generates an equal and opposite moment
		* Its point of action is the same direction from the pivot point
		* F<sub>n<sub>2</sub></sub> acts in the opposite direction of F<sub>g</sub>
	* F<sub>n<sub>1</sub></sub> sets ![sf](https://latex.codecogs.com/png.latex?\Sigma\:F=0)
		* F<sub>n<sub>1</sub></sub> = F<sub>g</sub> - F<sub>n<sub>2</sub></sub>
* F<sub>g</sub> behind both feet 
	* F<sub>g</sub> generates a net torque about the rear edge of the rear foot
	* F<sub>n<sub>1</sub></sub> and F<sub>n<sub>2</sub></sub> cannot act behind the rear edge of the rear foot
		* Neither can generate a counteracting torque
		* Biped falls over
* F<sub>g</sub> ahead of both feet 
	* Same as above, but F<sub>n</sub> can't get in front of the front foot
		
		
		
## References
* https://en.wikipedia.org/wiki/Support_polygon
* https://en.wikipedia.org/wiki/Convex_hull