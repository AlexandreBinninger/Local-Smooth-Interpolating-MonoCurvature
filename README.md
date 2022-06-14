# Smooth Interpolating Curves with Local Control and Monotonous Alternating Curvature

This folder contains the code related to the paper "Smooth Interpolating Curves with Local Control and Monotonous Alternating Curvature" by [Alexandre Binninger](https://alexandrebinninger.github.io) and [Olga Sorkine-Hornung](https://igl.ethz.ch/people/sorkine/index.php). The supplemental material is also made available here.

## Structure

This folder is composed of three subfolders:

- [```./matlab```](./matlab): to print the clodhoid shell (matlab)
- [```./extern```](./extern): the external libraries used by our method
- [```./code-gui```](./code-gui): the code with an integrated GUI (C++)
- *coming: a folder without GUI for easier integration into your own framework*

## matlab

Our matlab folder contains code to display the clothoid shell. The user can modifiy the starting point, angle and curvature directly in the code, and print more than one shell by uncommenting and completing the corresponding lines in ```main.m```. The code is kept very simple and self-explanatory.

## extern

This folder contains all the necessary additional libraries to run our method.

We use the following extern libraries:

- [libigl](https://libigl.github.io): a geometry processing library that we mainly use for the GUI it provides with ImGui.
- [libhedra](https://avaxman.github.io/libhedra/): an extension to libigl. We use its built-in functions to display curves and points as cylinders and spheres easily.
- [Clothoids](https://ebertolazzi.github.io/Clothoids/): a library dedicated to clothoid computations

Our code also relies on the [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) library.

## code-gui

In this folder, we include our code with the GUI interface. Tests have been performed on MacOS and Ubuntu.

### Compilation

To compile, use the following commands:

```mkdir build && cd build```

```cmake ..```

```make -j 4```

To execute the code, run

```./smooth-interpolating-monocurvature [input_file]```

The file ```[input_file]``` is an ***optional*** file path with the 2D coordinates of the represented shape. Example of data can be found in the ```/data``` folder. For instance, from the ```build``` folder,


```./smooth-interpolating-monocurvature ../data/guitar.txt```

It is not mandatory to use an input file to use the GUI, the default star shape would be displayed instead.


```./smooth-interpolating-monocurvature```

Various default shapes are available from the GUI.

### Use

We present how to use the provided interface. The video fully showcases its use. The GUI is separated in 2 parts. The left part deals with drawing options, such as colors, or informations to display, while the right part deals with actions that have an influence on the computation of the curve.

__View options__

It it possible to tweak several visual parameters of the curve, such as the color of the background, of control points or of the curve. It is also possible to change the size of the points and the width of the line. The width used for representing the curves integrates the user's zoom and does therefore not depend on it. The option `curvature color` gives different colors to control points given the sign of the curvature at these control points. It is possible to show/hide the control points, the curve, the control polygon, the tangents and the osculating circle. The curvature can also be displayed as a comb. Since it's representation does depend on the scale of the curve, the parameter "comb scale" allows the user to adjust the visibility of the comb. The option `heightmap color` gives a color to the comb related to its intensity (linearly but clamped to two times the media value).

The parameter `resolution` indicates the numbers of samples of the curves per control points. We use a global sampling method. Since our curves are parameterized by arc-length, it means that the number of samples is not the same between two control points. For instance, for 10 control points and a resolution of 100, the curve will be the result of 1000 samples evenly distributed on the curve. Two close points would have less samples between them than two distant points.

The parameter `frequency curvature` is the frequency of the comb lines of the curvature. A frequency of 5 means that every 5 sample, we draw a line of the curvature comb.

__Interactions__

It is possible to interact with the curve in several manners. For view options, the key 'C' displays the curvature comb, 'T' the tangents and 'O' the osculating circle.

To select a point, the user must hold SHIFT and click next to a control point. To allow multiple control points to be selected, the user can tick the checkbox `multi-selection` or hit the key 'M'. A selected point is displayed in cyan. The user can then hit the direction arrows on the keyboard to move the control points. Hitting 'R' will make it enter in rotation mode and points will rotate around their center if arrows on the keyboard are down (does not work if only one point is selected). 

To add control point, left click without moving the mouse. If there is only one selected control point, moving the mouse when the left click is down will move it accordingly. To delete a point, press 'D'. To deselect all the points, press 'S'. 

To add curvature or tangent constraints, select a unique point and press 'I' or click on button "Cons. modifier". Left click while moving the cursor. To only change the curvature but keep the same tangent, tick the box "only curvature". The program does not check the compatibility of the tangent and curvature with the existence of transitions, and properties ensured in the paper may not apply. To validate the constraint, press 'V' or click on "Validate Constraints". To remove tangent, curvature of one selected point, or to remove all constraints, click on the corresponding button.

__Curve Influence__

It is possible to tweak the parameters that influence the type of curves that are computed. On the top right of the GUI, it is possible to choose one model among the default ones. The line, circle, star and heart can produce a shape with a varying number of points. They are several random generators. The random method places point uniformly in a rectangle. Polygon random generate points such that the control polygon never self-intersects (except once between the first and last control point, if the curve is cycling), which is useful to try the non-intersecting method. The convex hull method places points such that the next point is not in the convex hull of the previous set of points. Finally, the Box Random method produces point such that the next point is not included in the axis-aligned bounding box. The figures named "Compare1", "Compare2" come from the paper "A Class of C2 interpolating splines." (Yuksel, 2020). The Omega figures come from the Clothoid Library we use. To erase all control points, click the button "erase". To print the control points positions, the tangents and the curvature at control point, click the button "print".

Under the "Spline" category, the user can choose which method to apply between 3-arcs clothoid transitions and clothoid-line-clothoid transitions. In both cases, the user can choose whether to use the Hermite G1 fitting problem for clothoid to estimate the curvature or not, and which increasing function between linear and max-linear to chose. In case of CLC transitions, the user can tick the box "3-arcs correction" to replace invalid CLC transitions with 3-arcs clothoids transitions (section 5.2). The intersection checkbox checks for intersection up to a certain number of neighbors (default is 5) and refines the curve accordingly (section 4.2.3).
Finally, the checkbox "cycle" determines whether the curve should be closed.

### Code organization

Our code is based on a Model-View-Controller pattern. All the relevant code for the method itself is located in folders [```./code-gui/src/model/```](./code-gui/src/model/) and [```./code-gui/src/utils/```](./code-gui/src/utils/). The main functions are in [```./code-gui/src/model/spline.cpp```](./code-gui/src/model/spline.cpp), functions `compute_3arcs_clothoid` and `compute_clothoid_line`.

### Data

The data files `bird.txt`, `face.txt` and `guitar.txt` come from the paper "A Class of C2 interpolating splines." (Yuksel, 2020).

<!--## Video

We provide two videos to exemplify our work:

- [results demo](./1-results-demo.mp4) for showcasing our results.
- [GUI demo](./2-GUI-demo.mp4) for showing how to interact with the program.
-->