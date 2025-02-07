# Path Object Intersection

A proof of concept tool to test the intersection of a path and an obstacle.

### Requires:
`eigen
googletest
python-matplotlib
`

# Usage
Functionality is provided as a library method:
```
bool Intersection::intersects(const Path& path, const Obstacle& obstacle)
```
Test suite is built into the executable `intersection_test`.
Also built is a `main` executable for direct usage (with a hardcoded case in `main.cpp`).

Finally, a python script is also provided for graphing and visual feedback of test cases.

## Discussion

As evidenced in the commit history, I initially started down a path of developing a discreet solution. I wanted to recursively split the path segment into smaller and smaller segments, testing the midpoint of each segment for containment within the vertices of the obstacle's polygon.  If any of the segment's bounding boxes were outside the bounding box of the obstacle, discard that segment and only recurse on the ones with intersecting bounding boxes.  I reasoned that for autonomous vehicle path simulation, having discreet points in the vicinity of an object to test not just intersection but probably many other useful parameters in the future would be advantageous.  However, this approach proved more cumbersome than time allowed, and ultimately I pivoted to an analytical solution, something I knew with confidence I could get working in limited time.  I've left my work-in-progress on the discreet solution here in the repository for historical purposes.

I ended up implementing a quadratic solution for line-circle intersection, paying special attention that the intersection point(s) must be within the start and end angles, and below the obstacle height.  Of particular concern for me was that the algorithm would perform as expected for very large positive and negative angles (ie `angle > 360 || angle < -360`).  If our path is defined as a dense spiral from `1080->2169` degrees, we still need to perform.  To accomplish this, I clamp my angles to the range `0 <= x < 360` for testing, taking care in the beyond-full-circule special cases where then `end_angle <= start_angle`.

This solution provided an efficient development approach, and avoided some of the pitfalls of the discreet solution.  For example, a discreen series of vertices might skip over a very narrow & pointy obstacle if the spatial resolution isn't fine enough.

## Visualization
I wrote a quick python script (`graph.py`) to interactively visualize the test cases in 3D using `matplotlib`.

<img src="https://github.com/user-attachments/assets/da234550-7448-4e20-8121-a0d863f3756d" width="250">
<img src="https://github.com/user-attachments/assets/874b7cb2-75d4-4994-9ea4-5f2d8d9fd972" width="250">
<img src="https://github.com/user-attachments/assets/4b1792f7-9fef-41ce-90d1-af730de5477f" width="250">
