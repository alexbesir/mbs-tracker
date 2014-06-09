MBS tracker
===========

Written by Alexander Besir (alex.besir@gmail.com), Faculty of Computer and Information Science Ljubljana, May, 2014

The MBS tracker is a freely-moving camera background subtraction based
object tracker written in Matlab. It utilizes the keyframe-based short-term
image stabilization technique to remove camera motion from the video,
MOG background subtraction and median image for foregorund extraction, and finally
hsitogram backprojection with CamShift for tracking.

The tracker may be seen in action on the author's web application:
http://alexbesir.com/projects/mbs/

Requirements:
-------------

- Matlab (recommended version r2013a or newer)
- OpenCV (version 2.4.8. or above)
- MexOpenCV (http://www.cs.stonybrook.edu/~kyamagu/mexopencv/)

Also, the tracker may be integrated to the VOT2013-toolkit
(http://www.votchallenge.net/vot2013/). The wrapper and all required
configuration files are provided int the `vot2013-toolkit-integration`
directory.

References:
-----------

- MBS_track function reference: https://github.com/alexbesir/mbs-tracker/blob/master/mbs-tracker/README.md
- VOT2013 toolkit inegration instructions: https://github.com/alexbesir/mbs-tracker/blob/master/vot2013-toolkit-integration/README.md
- Example output videos: http://www.alexbesir.com/projects/mbs/

License:
--------

The MIT License (MIT)

Copyright (c) 2014 Aleksander Besir

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
