#!/bin/sh

##################################################
################### parameters ###################
##################################################

results_directory=${1:?'Must specify directory where the results.bag is and in which the results will be outputed'}


echo "####################################################################################################"
echo "##### Generating results for path: ${results_directory}"
echo "####################################################################################################\n"


# generate CSVs from rosbag
rosrun robot_localization_tools bag2csv.sh ${results_directory}/results '/dynamic_robot_localization/localization_pose /tf /base_pose_ground_truth'



echo "\n======================================================================================="
echo "Building path (with arrows) from the ground truth and localization system poses"
rosrun robot_localization_tools path_plotter.py -i ${results_directory}/results_ground_truth_poses.txt+${results_directory}/results_localization_poses.txt -o ${results_directory}/paths -p 1 -v 8 -a 0.0025 -c 'g+b' -t 'Robot movement paths (green -> ground truth, blue -> localization system)' -s 1 -d 0 &



echo "\n======================================================================================="
echo "Building graphs of localization system results"

# Localization errors
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_error.csv -o ${results_directory}/translation_error_components_millimeters -x 2 -y '4+5+6' -w 0.5 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Translation error (millimeters)' -l 'Translation error in the x axis+Translation error in the y axis+Translation error in the z axis' -c 'y+g+b' -t 'Translation errors by axis' -g 1 -s 1 -d 0 &
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_error.csv -o ${results_directory}/translation_error_millimeters -x 2 -y '7' -w 0.5 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Translation error (millimeters)' -l 'Translation error' -c 'b' -t 'Translation error' -g 1 -s 1 -d 0 &
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_error.csv -o ${results_directory}/rotation_error_axis -x 2 -y '8+9+10' -w 0.5 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Rotation error axis component' -l 'Rotation error x axis component+Rotation error y axis component+Rotation error z axis component' -c 'y+g+b' -t 'Rotation error axis' -g 1 -s 1 -d 0 &
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_error.csv -o ${results_directory}/rotation_error_degrees -x 2 -y '11' -w 0.5 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Rotation error (degrees)' -l 'Rotation error' -c 'b' -t 'Rotation error' -g 1 -s 1 -d 0 &

# Localization corrections
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_detailed.csv -o ${results_directory}/translation_corrections_components_millimeters -x 2 -y '47+48+49' -w 0.5 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Translation correction (millimeters)' -l 'Translation correction in the x axis+Translation correction in the y axis+Translation correction in the z axis' -c 'y+g+b' -t 'Translation corrections by axis' -g 1 -s 1 -d 0 &
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_detailed.csv -o ${results_directory}/translation_correction_millimeters -x 2 -y '50' -w 0.5 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Translation correction (millimeters)' -l 'Translation correction' -c 'b' -t 'Translation correction' -g 1 -s 1 -d 0 &
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_detailed.csv -o ${results_directory}/rotation_correction_axis -x 2 -y '51+52+53' -w 0.5 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Rotation correction axis component' -l 'Rotation correction x axis component+Rotation correction y axis component+Rotation correction z axis component' -c 'y+g+b' -t 'Rotation correction axis' -g 1 -s 1 -d 0 &
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_detailed.csv -o ${results_directory}/rotation_correction_degrees -x 2 -y '54' -w 0.5 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Rotation correction (degrees)' -l 'Rotation correction' -c 'b' -t 'Rotation correction' -g 1 -s 1 -d 0 &

# Registration analysis
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_detailed.csv -o ${results_directory}/outlier_percentage -x 2 -y '55' -w 0.5 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Outlier percentage [0..1]' -l 'Outlier percentage' -c 'b' -t 'Outlier percentage' -g 1 -s 1 -d 0 &
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_detailed.csv -o ${results_directory}/root_mean_square_error -x 2 -y '56' -w 0.5 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Root mean square error (millimeters)' -l 'Root mean square error' -c 'b' -t 'Root mean square error' -g 1 -s 1 -d 0 &
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_detailed.csv -o ${results_directory}/registered_points -x 2 -y '57+58' -w 0.5 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Number of points' -l 'Number registered inliers+Number registered points (inliers and outliers)' -c 'g+b' -t 'Number registered points' -g 1 -s 1 -d 0 &
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_detailed.csv -o ${results_directory}/registered_points_angular_distribution -x 2 -y '59+60' -w 0.5 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Angular distribution percentage [0..1]' -l 'Inliers angular distribution+Outliers angular distribution' -c 'g+b' -t 'Registered points angular distribution' -g 1 -s 1 -d 0 &

# Number of points after several processing stages
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_diagnostics.csv -o ${results_directory}/pointclouds_size -x 2 -y '4+5+6+7+8+9+10' -w 0.5 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Pointcloud size' -l 'Number points reference pointcloud+Number points reference pointcloud after filtering+Number keypoints reference pointcloud+Number points ambient pointcloud+Number points ambient pointcloud after filtering+Number points ambient pointcloud used in registration+Number keypoints ambient pointcloud' -c '#7a643b+#708381+#f17008+#f17008+#eeb111+#b00007+#16a111' -t 'Pointclouds size in several processing stages' -g 0 -s 1 -d 0 &

# Processing time of the main processing stages
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_times.csv -o ${results_directory}/computation_times_milliseconds -x 2 -y '4+5+6+7+8+9+10+11+12' -w 0.5 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Computation time (milliseconds)' -l 'Global time+Filtering time+Surface normal estimation time+Keypoint selection time+Initial pose estimation time+Pointcloud registration time+Outlier detection time+Registered points angular distribution analysis time+Transformation validators time' -c '#7a643b+#708381+#f17008+#f17008+#eeb111+#b00007+#16a111+#1869c5+#9426b2' -t 'Computation times' -g 1 -s 1 -d 0 &



#echo "\n======================================================================================="
#echo "Fitting probability distributions to localization system results"
#rosrun robot_localization_tools probability_distribution_fitter.py -i ${results_directory}/data_ct.csv -o ${results_directory}/ct -s 1 -d 1 -c 3 -b 1 -x Error -t 'Translation Error' -f 1


wait


echo "\n####################################################################################################"
echo "##### Finished generating results for ${results_directory}"
echo "####################################################################################################\n"
