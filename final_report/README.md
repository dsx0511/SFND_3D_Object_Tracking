# Final Report

## FP.5 Performance Evaluation 1

The tables below show a sequence of images with TTC estimation and its corresponding Lidar points in BEV, where an implausible Lidar TTC estimation was observed. The TTC Lidar in this sequence shows a very strong fluctuation of TTC between 9.34s and 34.34s, whereas the sequence is only 0.4s long. This can be attributed to the constant velocity model, where only the distance difference between current and previous frame is considered. In the real world scenario with complex combination between acceleration and brake degree, the constant velocity model is not able to fit this motion and causes the instability of TTC.

### TTC images
| frame | TTC image |
| ----- |    :-:    |
| 1 | <img src="FP_5/TTC/06.png" height="150" /> |
| 2 | <img src="FP_5/TTC/07.png" height="150" /> |
| 3 | <img src="FP_5/TTC/08.png" height="150" /> |
| 4 | <img src="FP_5/TTC/09.png" height="150" /> |

### BEV images
| frame | BEV image |
| ----- |    :-:    |
| 0 | <img src="FP_5/BEV/05.png" height="500" /> |
| 1 | <img src="FP_5/BEV/06.png" height="500" /> |
| 2 | <img src="FP_5/BEV/07.png" height="500" /> |
| 3 | <img src="FP_5/BEV/08.png" height="500" /> |
| 4 | <img src="FP_5/BEV/09.png" height="500" /> |

## FP.6 Performance Evaluation 2

