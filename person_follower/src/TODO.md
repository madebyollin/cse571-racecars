# TODO for PersonFollower:

#### better code stucture 

- [ ] take our main algorithm out of ```__init__``` 
- [ ] launch file to launch multi nodes
- [ ] fix conflict between joystick and our personfollow algorithm (don't send servo order at the same time)


#### add longitudinal control

- [ ] Check the size of the current bounding box and drive forward or backwards by a small amount in order to achieve a target size *EDIT:* or use the depth camera masked to the bounding-box region

#### create new nodes

- [ ] One for publishing "person boxes"
- [x] One for figuring out the delta_angle that we should give to servo controller to make sure the box is right at the center of the image, also the speed&duration that we should give to the motor controller to make sure the size of bouding box stay the same(which mean the distance between car and person keep constant)
- [ ] One for the obstacle avoidance 
- [ ] One Master Node for considering both following person and obstacle avoidance

#### Questions

- [ ] do we need to distingush different people?
