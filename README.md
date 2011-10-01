[Tsai's Camera Calibration](http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=1087109&tag=1)
-------------------------

Build
-----

`$ g++ calibrate.cc -I/opt/local/include/opencv -L/opt/local/lib -lcv -lhighgui -lcxcore -Wall -o calibrate`

Run
---

* Interactive Mode: `$ ./calibrate`
* Batch Mode: `$ ./calibrate <filenames>`

Documents
---------

* [Report](https://github.com/downloads/vbajpai/cameracalibration/report.pdf)
