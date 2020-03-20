# ASL project
#### Building  
I fixed some minor things to get this to compile.
Now you can run it as 
```
./build/cpp/fastslam1/fastslam1_simulation ./cpp/core/example_webmap.mat'
```
or other input files of course.
Note that the inputfile `example_webmap_simple.mat` didn't work for me...

#### Documentation
I have created a doxyfile that generates call and callee graphs.
Navigate to the root of the submodule and run `doxygen doc/Doxyfile`.
Open `doc/html/index.html` in your browser, open a file (e.g. the `main.cpp` in the fastslam1 dir) and check out the function relations.

**Check out the call graph**  
![fasterslam call graph][fasterslam-call-graph]

FastSLAM
========

Retrieval

Prediction

Measurement Update

Importance Weight

Resampling

[fasterslam-call-graph]: doc/fasterslam_call_graph.png "fasterslam call graph"
