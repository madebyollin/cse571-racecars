# TODO for PersonFollower:

#### better code stucture 

- [ ] take our main algorithm out of ```__init__``` 
- [ ] launch file to launch multi nodes
- [x] fix conflict between joystick and our personfollow algorithm (don't send servo order at the same time)


#### add longitudinal control

- [x] **Level 0:** Check the size of the current bounding box and drive forward or backwards by a small amount in order to achieve a target size
- [x] **Level 1:** use the depth camera masked to the bounding-box region to compute a current depth, and adjust the speed towards a target depth.

#### cross-frame tracking

- [x] **Level 0:** Always track the largest box
- [x] **Level 1:** Use area-of-intersection to compute which box corresponds to the tracked box from the previous frame
- [ ] **Level 2:** Use a kalman filter (or hand-coded filter) to estimate position if the bounding box is no longer visible
- [ ] **Bonus:** Use visual features compute box correspondences

#### create new nodes

- [x] One for publishing "person boxes"
- [x] One for figuring out the delta_angle that we should give to servo controller to make sure the box is right at the center of the image, also the speed&duration that we should give to the motor controller to make sure the size of bouding box stay the same(which mean the distance between car and person keep constant)
- [ ] One for the obstacle avoidance 
- [ ] One Master Node for considering both following person and obstacle avoidance

#### Questions

- [x] do we need to distingush different people?
  - Probably yes - appearance averaging code should do this
