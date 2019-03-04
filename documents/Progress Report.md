\begin{centering}

{\Large{\textbf{571A Progress Report}}}

\small{Nansong Yi, Thomas Wang, Ollin Boer Bohan}

\end{centering}

## Successes

Our current version of the person following software is successful at tracking and following people under ideal conditions. The software uses YOLO for 25hz bounding box detection, bounding box size to select an initial person to follow, and bounding-box overlap metrics to track that person across time. It also uses values from the depth camera to move closer or further from the tracked person in order to maintain its set distance.

[You can see a version of our system being tested here.](https://youtu.be/2rFWG4xlKyY)

We've since improved the robustness slightly, but the underlying functionality is identical.

## Unforeseen Problems

 After working with the actual robot and testing our initial system, our priorities have shifted slightly. In particular:

* Advanced (e.g. map-based) obstacle avoidance seems less important, because, for the most part, humans will not walk through obstacles, so following the human closely is unlikely to lead the robot into obstacles. Additionally, the optimal camera angle for person detection (looking upwards) is not well-suited for obstacle avoidance. We still intend to implement some basic obstacle avoidance (possibly using the LIDAR, or just detecting immobility), but consider it a less essential feature.
* Cross-frame tracking (likely with per-person feature descriptors as well as trajectory estimates) seems much more important–the bounding boxes for the target person can often flicker or vanish, and there are frequently spurious matches (from nearby people, or things that look like people) which can confuse the robot. Having a more intelligent cross-frame tracking system seems essential for a good demo (since the occasional lapses in tracking are the primary way in which the robot appears to malfunction), so we will probably focus on this.
* In addition, it may be useful to have some sort of indicator to show when the robot is successfully tracking and when it is lost–right now it's not always clear, and this would definitely help the user interact with the robot more easily.

## Timeline

- **Week 8:**
    - Improve cross-frame tracking to incorporate a motion model (and possibly feature descriptors) so that tracking robustness improves and the robot can consistently follow a single person.
    - Implement basic obstacle avoidance (detecting immobility via odometry)
- **Week 9:**
    - Continue improving cross-frame tracking and obstacle avoidance
    - Possibly add a "successful tracking" indicator of some sort.
    - Perform testing to evaluate our system (measuring how frequently it loses tracking, how frequently it gets stuck, and what the causes are).
    - Record and edit demo video
- **Week 10:**
    - Prepare poster
    - Prepare final report