# Final Report

## FP.1 Match 3D Objects
The algorithm start looping over all bounding boxes in the current frame. For every bounding box in current frame, it first loops over all matched keypoint pairs. If the keypoint in current frame lies in this bounding box, the matching `cv::DMatch` is appended to this bounding box in `BoundingBox::kptMatches`. After finding all matchings with current keypoint lies in this box, it loops over all bounding boxes in the previous frame, and calculates the number of matched keypoints tha lies in every bounding box in the previous frame. The bounding box in the previous frame with most matched keypoints is marked as a matching of the bounding box in the current frame. This procedure is repeated for all remaining bounding boxes in the current frame.

## FP.2 Compute Lidar-based TTC
The Lidar TTC is computed with the same way as in the exercise of lesson 3 using the formula `TTC = minXCurr * dT / (minXPrev - minXCurr)`. For the statistical robustness, the mean $\mu$ and variance $\sigma$ of the distribution in x-axis is calculated. After that, the nearest point in x-direction within $\alpha*\sigma$ is considered as nearest point, where $\alpha$ is a hyperparameter. Points outside this range are considered as outliers.

## FP.3 Associate Keypoint Correspondences with Bounding Boxes
In this implementation, keypoints and keypoint matches are already appended to the vector `BoundingBox::kptMatches` in the function `matchBoundingBoxes`. Therefore, the function `clusterKptMatchesWithROI` only conducts an outlier filtering. To address the outliers, a check is performed in this function. It calculates the median of distances between matched keypoints in both frames $\tilde{x}$ and defines a tolerance rate $t$. If the distance of a matching is out of the range $[\tilde{x}-t, \tilde{x}+t]$, this matching is removed.

## FP.4 Compute Camera-based TTC
The Camera TTC is computed with the same way as in the exercise of lesson 3 using the formula `TTC = -dT / (1 - medDistRatio)`. It calculates the distance between every keypoint pair in the current as well as the distance between both matched keypoints in the previous frame. The distance ratios are recorded in a vector. After looping over all pairs, the median of all distance ratios is used to calculate the camera TTC.

## FP.5 Performance Evaluation 1

The tables below show a sequence of images with TTC estimation and its corresponding Lidar points in BEV, where an implausible Lidar TTC estimation was observed. The TTC Lidar in this sequence shows a very strong fluctuation of TTC between 9.34s and 34.34s, whereas the sequence is only 0.4s long and no rapid acceleration is observed. This can be attributed to the constant velocity model, where only the distance difference between current and previous frame is considered. In the real world scenario with complex combination between acceleration and brake degree, the constant velocity model is not able to fit this motion and causes the instability of TTC.

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

The camera TTC of all detector & descriptor combinations can be found in the [csv](./FP_6/results.csv) file. After that, the line charts below show the TTC of different keypoint detectors over time for every keypoint descriptor respectively. As Harris and ORB detector always produce unreliable results, e.g. -inf, Nan, unsuccess matchings (fewer frames recorded in the table), we omit them in all graphs.

It seems that BRISK detector is sometimes way off due to the fluctuations in TTC estimation, which again can be attributed to the constant velocity model discussed before. However, using an average or a median of distances between every matched keypoint pairs could increase the stability compared to using the single nearest point in Lidar TTC. This can be proofed with some other detector & descriptor combinations in camera TTC, where a smoother line can be observed. For example, the FAST and AKAZE detector produced smoothest line with almost all descriptors. The best combinations might be FAST+SIFT, AKAZE+AKAZE and AKAZE+SIFT. However, this should further be validated using some accuracy metrics.

<img src="FP_6/BRISK.png" />
<img src="FP_6/BRIEF.png" />
<img src="FP_6/ORB.png" />
<img src="FP_6/FREAK.png" />
<img src="FP_6/AKAZE.png" />
<img src="FP_6/SIFT.png" />