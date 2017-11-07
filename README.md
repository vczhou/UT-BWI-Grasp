# UT-BWI-Grasp

The purpose of this package is twofold: first, we would like to change the BWIBot's arm to use gpd instead of agile grasp. Second, we would
like to train the neural network that gpd uses in order to better fit our arm's specifications.

To Do:
- write a package that runs GPD with MoveIt to pick up objects and stores the grasps that it tried along with their success state
- collect data by running this package multiple times on multiple objects
- train the network with the data
