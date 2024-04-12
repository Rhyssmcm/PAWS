PAWS Quadruped Interactive Kinematic Model, Trajectory Generator and Locomotion simulation in heighfield and incline plane on Pybullet

Kinematic model: Kenmatic Model allows you to control PAWS Pitch, roll, height and step length freely and can return the joint positions for each leg.

Parabolic Arc Generator: This produces a Parabolic Arc for a quadruped trajectory Swing phase and straight line Stance Phase. Produces 800 coordinates for entire cycle,: saved as a CSV file and produces trajectory graph. 
Input step height (m) and Step Length(m) where the step length input is distance from 0 to +-stepheight.

Bezier Curve Generator: This produces a Piecewise Bezier Curve for a quadruped trajectory Swing phase and straight line Stance Phase. Produces 800 coordinates for entire cycle,: saved as a CSV file and produces trajectory graph.
Input speed of quadruped (mm/s), stride duration,  Swing Height and Stance height (mm) and quadruped height.

leg swing/stance angle generator: Class taht takes the desired swing and stance coordinates, roll and pitch to generate the joint angles for each trajectory.

leg trot: Class that places the swing and stance joint positions onto the correct quadruped legs.

PAWS File includes data needed for launching PAWS in Pybullet including URDF for PAWS and plane.
