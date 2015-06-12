#!/usr/bin/env sh

##################################################
################### parameters ###################
##################################################

results_directory=${1:?'Must specify directory where the results.bag is and in which the results will be saved'}
extract_cvs_from_bag=${2:-true}
use_odometry=${3:-true}

echo "############################################################################################################################################################"
echo "##### Generating results for path: ${results_directory}"
echo "############################################################################################################################################################\n"


# generate CSVs from rosbag
if [ "${extract_cvs_from_bag}" = true ]; then
	rosrun robot_localization_tools bag2csv.sh ${results_directory}/results '/dynamic_robot_localization/diagnostics /dynamic_robot_localization/localization_detailed /dynamic_robot_localization/localization_error /dynamic_robot_localization/odometry_error /dynamic_robot_localization/localization_times /rosout'
	
	mkdir -p "${results_directory}/pdf"
	mkdir -p "${results_directory}/svg"
	mkdir -p "${results_directory}/eps"
fi



echo "\n======================================================================================="
echo "Building path (with arrows) from the ground truth and localization system poses"


path_files="${results_directory}/results_ground_truth_poses.txt+${results_directory}/results_localization_poses.txt"
rosrun robot_localization_tools path_plotter.py -i ${path_files} -o ${results_directory}/robot-movement-path -p 1 -v 8 -a 0.01 -c 'g+b' -t 'Movement path (green -> ground truth, blue -> localization system)' -s 1 -q 1 -d 0 &

if [ "${use_odometry}" = true ]; then
	path_files=${path_files}+"${results_directory}/results_odometry_poses.txt"
	rosrun robot_localization_tools path_plotter.py -i ${path_files} -o ${results_directory}/robot-movement-path-with-odometry -p 1 -v 8 -a 0.01 -c 'g+b+r' -t 'Movement path (green -> ground truth, blue -> localization system, red -> odometry)' -s 1 -q 1 -d 0 &
fi

echo "\n======================================================================================="
echo "Building graphs for localization system results"

graphs_common_configs="-k 0.75 -r 1 -g 1 -s 1 -q 1 -d 0"
path_graphs_common_configs="--reset 1 --grid 1 -k 0.75 -r 10 -g 3 -s 1 -q 1 -d 0"

# Evolution of x, y and z
if [ "${use_odometry}" = true ]; then
	rosrun robot_localization_tools graph_plotter.py -i ${path_files} -o ${results_directory}/robot-movement-path-position-evolution-x -x '0-0-0' -y '1-1-1' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'X position (meters)' -l 'Ground truth x positions+Localization system x positions+Odometry x positions' -c 'g+b+r' -t 'Evolution of x position' ${graphs_common_configs} &
	rosrun robot_localization_tools graph_plotter.py -i ${path_files} -o ${results_directory}/robot-movement-path-position-evolution-y -x '0-0-0' -y '2-2-2' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Y position (meters)' -l 'Ground truth y positions+Localization system y positions+Odometry y positions' -c 'g+b+r' -t 'Evolution of y position' ${graphs_common_configs} &
	rosrun robot_localization_tools graph_plotter.py -i ${path_files} -o ${results_directory}/robot-movement-path-position-evolution-z -x '0-0-0' -y '3-3-3' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Z position (meters)' -l 'Ground truth z positions+Localization system z positions+Odometry z positions' -c 'g+b+r' -t 'Evolution of z position' ${graphs_common_configs} &
else
	rosrun robot_localization_tools graph_plotter.py -i ${path_files} -o ${results_directory}/robot-movement-path-position-evolution-x -x '0-0' -y '1-1' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'X position (meters)' -l 'Ground truth x positions+Localization system x positions' -c 'g+b' -t 'Evolution of x position' ${graphs_common_configs} &
	rosrun robot_localization_tools graph_plotter.py -i ${path_files} -o ${results_directory}/robot-movement-path-position-evolution-y -x '0-0' -y '2-2' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Y position (meters)' -l 'Ground truth y positions+Localization system y positions' -c 'g+b' -t 'Evolution of y position' ${graphs_common_configs} &
	rosrun robot_localization_tools graph_plotter.py -i ${path_files} -o ${results_directory}/robot-movement-path-position-evolution-z -x '0-0' -y '3-3' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Z position (meters)' -l 'Ground truth z positions+Localization system z positions' -c 'g+b' -t 'Evolution of z position' ${graphs_common_configs} &
fi

# Cumulative position evolution
if [ "${use_odometry}" = true ]; then
	rosrun robot_localization_tools path_velocity_and_acceleration_plotter.py -i ${path_files} -f -1 -o ${results_directory}/robot-movement-path-position-differences -p 1 -x '0-0-0' -y '1-1-1' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Position differences from consecutive poses (meters)' -l 'Ground truth+Localization system+Odometry' -c 'g+b+r' -t 'Position differences from consecutive poses' ${path_graphs_common_configs} &
	rosrun robot_localization_tools path_velocity_and_acceleration_plotter.py -i ${path_files} -f -1 -o ${results_directory}/robot-movement-path-angular-differences -p 0 -x '0-0-0' -y '4-4-4' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Angular differences from consecutive poses (degrees)' -l 'Ground truth+Localization system+Odometry' -c 'g+b+r' -t 'Angular differences from consecutive poses' ${path_graphs_common_configs} &
	rosrun robot_localization_tools path_velocity_and_acceleration_plotter.py -i ${path_files} -f 0 -o ${results_directory}/robot-movement-path-position-cumulative -p 1 -x '0-0-0' -y '1-1-1' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Cumulative position differences from consecutive poses (meters)' -l 'Ground truth+Localization system+Odometry' -c 'g+b+r' -t 'Cumulative position differences from consecutive poses' ${path_graphs_common_configs} &
	rosrun robot_localization_tools path_velocity_and_acceleration_plotter.py -i ${path_files} -f 0 -o ${results_directory}/robot-movement-path-angular-cumulative -p 0 -x '0-0-0' -y '4-4-4' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Cumulative angular differences from consecutive poses (degrees)' -l 'Ground truth+Localization system+Odometry' -c 'g+b+r' -t 'Cumulative angular differences from consecutive poses' ${path_graphs_common_configs} &
else
	rosrun robot_localization_tools path_velocity_and_acceleration_plotter.py -i ${path_files} -f -1 -o ${results_directory}/robot-movement-path-position-differences -p 1 -x '0-0' -y '1-1' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Position differences from consecutive poses (meters)' -l 'Ground truth+Localization system' -c 'g+b' -t 'Position differences from consecutive poses' ${path_graphs_common_configs} &
	rosrun robot_localization_tools path_velocity_and_acceleration_plotter.py -i ${path_files} -f -1 -o ${results_directory}/robot-movement-path-angular-differences -p 0 -x '0-0' -y '4-4' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Angular differences from consecutive poses (degrees)' -l 'Ground truth+Localization system' -c 'g+b' -t 'Angular differences from consecutive poses' ${path_graphs_common_configs} &
	rosrun robot_localization_tools path_velocity_and_acceleration_plotter.py -i ${path_files} -f 0 -o ${results_directory}/robot-movement-path-position-cumulative -p 1 -x '0-0' -y '1-1' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Cumulative position differences from consecutive poses (meters)' -l 'Ground truth+Localization system' -c 'g+b' -t 'Cumulative position differences from consecutive poses' ${path_graphs_common_configs} &
	rosrun robot_localization_tools path_velocity_and_acceleration_plotter.py -i ${path_files} -f 0 -o ${results_directory}/robot-movement-path-angular-cumulative -p 0 -x '0-0' -y '4-4' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Cumulative angular differences from consecutive poses (degrees)' -l 'Ground truth+Localization system' -c 'g+b' -t 'Cumulative angular differences from consecutive poses' ${path_graphs_common_configs} &
fi

if [ "${use_odometry}" = true ]; then
# Velocity
	rosrun robot_localization_tools path_velocity_and_acceleration_plotter.py -i ${path_files} -f 1 -o ${results_directory}/robot-movement-path-linear-velocity -p 1 -x '0-0-0' -y '1-1-1' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Linear velocity (meters / second)' -l 'Ground truth+Localization system+Odometry' -c 'g+b+r' -t 'Estimated linear velocity' ${path_graphs_common_configs} &
	rosrun robot_localization_tools path_velocity_and_acceleration_plotter.py -i ${path_files} -f 1 -o ${results_directory}/robot-movement-path-angular-velocity -p 0 -x '0-0-0' -y '4-4-4' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Angular velocity (degrees / second)' -l 'Ground truth+Localization system+Odometry' -c 'g+b+r' -t 'Estimated angular velocity' ${path_graphs_common_configs} &
else
	rosrun robot_localization_tools path_velocity_and_acceleration_plotter.py -i ${path_files} -f 1 -o ${results_directory}/robot-movement-path-linear-velocity -p 1 -x '0-0' -y '1-1' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Linear velocity (meters / second)' -l 'Ground truth+Localization system' -c 'g+b' -t 'Estimated linear velocity' ${path_graphs_common_configs} &
	rosrun robot_localization_tools path_velocity_and_acceleration_plotter.py -i ${path_files} -f 1 -o ${results_directory}/robot-movement-path-angular-velocity -p 0 -x '0-0' -y '4-4' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Angular velocity (degrees / second)' -l 'Ground truth+Localization system' -c 'g+b' -t 'Estimated angular velocity' ${path_graphs_common_configs} &
fi


# Acceleration
if [ "${use_odometry}" = true ]; then
	rosrun robot_localization_tools path_velocity_and_acceleration_plotter.py -i ${path_files} -f 2 -o ${results_directory}/robot-movement-path-linear-acceleration -p 1 -x '0-0-0' -y '1-1-1' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Linear acceleration (meters / second / second)' -l 'Ground truth+Localization system+Odometry' -c 'g+b+r' -t 'Estimated linear acceleration' ${path_graphs_common_configs} &
	rosrun robot_localization_tools path_velocity_and_acceleration_plotter.py -i ${path_files} -f 2 -o ${results_directory}/robot-movement-path-angular-acceleration -p 0 -x '0-0-0' -y '4-4-4' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Angular acceleration (degrees / second / second)' -l 'Ground truth+Localization system+Odometry' -c 'g+b+r' -t 'Estimated angular acceleration' ${path_graphs_common_configs} &
else
	rosrun robot_localization_tools path_velocity_and_acceleration_plotter.py -i ${path_files} -f 2 -o ${results_directory}/robot-movement-path-linear-acceleration -p 1 -x '0-0' -y '1-1' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Linear acceleration (meters / second / second)' -l 'Ground truth+Localization system' -c 'g+b' -t 'Estimated linear acceleration' ${path_graphs_common_configs} &
	rosrun robot_localization_tools path_velocity_and_acceleration_plotter.py -i ${path_files} -f 2 -o ${results_directory}/robot-movement-path-angular-acceleration -p 0 -x '0-0' -y '4-4' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Angular acceleration (degrees / second / second)' -l 'Ground truth+Localization system' -c 'g+b' -t 'Estimated angular acceleration' ${path_graphs_common_configs} &
fi


# Localization errors
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_error.csv -o ${results_directory}/translation-error-components-millimeters -x 2 -y '4+5+6' -w 0.25 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Translation error (millimeters)' -l 'Translation error in the x axis+Translation error in the y axis+Translation error in the z axis' -c 'y+g+b' -t 'Translation errors by axis' ${graphs_common_configs} &
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_error.csv -o ${results_directory}/translation-error-millimeters -x 2 -y '7' -w 0.25 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Translation error (millimeters)' -l 'Translation error' -c 'b' -t 'Translation error' ${graphs_common_configs} &
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_error.csv -o ${results_directory}/rotation-error-axis -x 2 -y '8+9+10' -w 0.25 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Rotation error axis component [0..1]' -l 'Rotation error x axis component+Rotation error y axis component+Rotation error z axis component' -c 'y+g+b' -t 'Rotation error axis' ${graphs_common_configs} &
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_error.csv -o ${results_directory}/rotation-error-degrees -x 2 -y '11' -w 0.25 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Rotation error (degrees)' -l 'Rotation error' -c 'b' -t 'Rotation error' ${graphs_common_configs} &

# Odometry errors
if [ "${use_odometry}" = true ]; then
	rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_odometry_error.csv -o ${results_directory}/odometry-translation-error-components-millimeters -x 2 -y '4+5+6' -w 0.25 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Translation error (millimeters)' -l 'Translation error in the x axis+Translation error in the y axis+Translation error in the z axis' -c 'y+g+b' -t 'Translation errors by axis' ${graphs_common_configs} &
	rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_odometry_error.csv -o ${results_directory}/odometry-translation-error-millimeters -x 2 -y '7' -w 0.25 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Translation error (millimeters)' -l 'Translation error' -c 'b' -t 'Translation error' ${graphs_common_configs} &
	rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_odometry_error.csv -o ${results_directory}/odometry-rotation-error-axis -x 2 -y '8+9+10' -w 0.25 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Rotation error axis component [0..1]' -l 'Rotation error x axis component+Rotation error y axis component+Rotation error z axis component' -c 'y+g+b' -t 'Rotation error axis' ${graphs_common_configs} &
	rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_odometry_error.csv -o ${results_directory}/odometry-rotation-error-degrees -x 2 -y '11' -w 0.25 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Rotation error (degrees)' -l 'Rotation error' -c 'b' -t 'Rotation error' ${graphs_common_configs} &
fi

# Localization corrections
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_detailed.csv -o ${results_directory}/translation-corrections-components-millimeters -x 2 -y '11+12+13' -w 0.25 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Translation correction (millimeters)' -l 'Translation correction in the x axis+Translation correction in the y axis+Translation correction in the z axis' -c 'y+g+b' -t 'Translation corrections by axis' ${graphs_common_configs} &
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_detailed.csv -o ${results_directory}/translation-correction-millimeters -x 2 -y '14' -w 0.25 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Translation correction (millimeters)' -l 'Translation correction' -c 'b' -t 'Translation correction' ${graphs_common_configs} &
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_detailed.csv -o ${results_directory}/rotation-correction-axis -x 2 -y '15+16+17' -w 0.25 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Rotation correction axis component [0..1]' -l 'Rotation correction x axis component+Rotation correction y axis component+Rotation correction z axis component' -c 'y+g+b' -t 'Rotation correction axis' ${graphs_common_configs} &
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_detailed.csv -o ${results_directory}/rotation-correction-degrees -x 2 -y '18' -w 0.25 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Rotation correction (degrees)' -l 'Rotation correction' -c 'b' -t 'Rotation correction' ${graphs_common_configs} &

# Registration analysis
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_detailed.csv -o ${results_directory}/outlier-percentage -x 2 -y '19' -w 0.25 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Outlier percentage [0..1]' -l 'Outlier percentage' -c 'b' -t 'Outlier percentage' ${graphs_common_configs} &
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_detailed.csv -o ${results_directory}/root-mean-square-error-inliers -x 2 -y '20' -w 0.25 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Root mean square error (millimeters)' -l 'Root mean square error' -c 'b' -t 'Root mean square error of inliers' ${graphs_common_configs} &
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_detailed.csv -o ${results_directory}/registered-points -x 2 -y '21+22' -w 0.25 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Number of points' -l 'Number of registered inliers+Number of registered points (inliers and outliers)' -c 'g+b' -t 'Number registered points' ${graphs_common_configs} &
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_detailed.csv -o ${results_directory}/registered-points-angular-distribution -x 2 -y '23+24' -w 0.25 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Angular distribution percentage [0..1]' -l 'Inliers angular distribution+Outliers angular distribution' -c 'g+b' -t 'Registered points angular distribution' ${graphs_common_configs} &

# Number of points after several processing stages
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_diagnostics.csv -o ${results_directory}/pointclouds-size -x 2 -y '4+5+6+7+8+9+10' -w 0.25 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Pointcloud size' -l 'Number of points in reference pointcloud+Number of points in reference pointcloud after filtering+Number of keypoints in reference pointcloud+Number of points in ambient pointcloud+Number of points in ambient pointcloud after filtering+Number of points in ambient pointcloud used in registration+Number of keypoints in ambient pointcloud' -c '#7a643b+#708381+#f17008+#f17008+#eeb111+#b00007+#16a111' -t 'Pointclouds size in several processing stages' -r 1 -g 0 -s 1 -q 1 -d 0 &

# Processing time of the main processing stages
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_times.csv -o ${results_directory}/computation-times-milliseconds -x 2 -y '4+5+6+7+8+9+10+11+12' -w 0.25 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Computation time (milliseconds)' -l 'Global time+Filtering time+Surface normal estimation time+Keypoint selection time+Initial pose estimation time+Pointcloud registration time+Outlier detection time+Registered points angular distribution analysis time+Transformation validators time' -c '#7a643b+#708381+#f17008+#f17008+#eeb111+#b00007+#16a111+#1869c5+#9426b2' -t 'Computation times' ${graphs_common_configs} &



echo "\n======================================================================================="
echo "Fitting probability distributions to localization system results"

probability_distributions_csv="${results_directory}/probability_distributions_temp.csv"
echo -n "" > ${probability_distributions_csv}

probability_distributions_common_configs="-b -1 -n 100 -m -1 -l 11 -a 10 -w 0.25 -g 4 -r 1 -s 1 -q 1 -d 0 -f 0"

# Localization errors
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_error.csv -o ${results_directory}/translation-error-millimeters-distributions -c 7 -t 'Probability distributions for translation error (millimeters)' -x 'Translation error histogram bins (millimeters)' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_error.csv -o ${results_directory}/rotation-error-degrees-distributions -c 11 -t 'Probability distributions for rotation error (degrees)' -x 'Rotation error histogram bins (degrees)' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &

# Odometry errors
if [ "${use_odometry}" = true ]; then
	rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_odometry_error.csv -o ${results_directory}/odometry-translation-error-millimeters-distributions -c 7 -t 'Probability distributions for translation error (millimeters)' -x 'Translation error histogram bins (millimeters)' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
	rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_odometry_error.csv -o ${results_directory}/odometry-rotation-error-degrees-distributions -c 11 -t 'Probability distributions for rotation error (degrees)' -x 'Rotation error histogram bins (degrees)' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
fi

# Localization corrections
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_detailed.csv -o ${results_directory}/translation-correction-millimeters-distributions -c 14 -t 'Probability distributions for translation correction (millimeters)' -x 'Translation correction histogram bins (millimeters)' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_detailed.csv -o ${results_directory}/rotation-correction-degrees-distributions -c 18 -t 'Probability distributions for rotation correction (degrees)' -x 'Rotation correction histogram bins (degrees)' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &

# Registration analysis
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_detailed.csv -o ${results_directory}/outlier-percentage-distributions -c 19 -t 'Probability distributions for outlier percentage [0..1]' -x 'Outlier percentage histogram bins [0..1]' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_detailed.csv -o ${results_directory}/root-mean-square-error-inliers-distributions -c 20 -t 'Probability distributions for inliers root mean square error (millimeters)' -x 'Root mean square error histogram bins (millimeters)' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_detailed.csv -o ${results_directory}/registered-points-angular-distribution-inliers-distributions -c 23 -t 'Probability distributions for inliers angular distribution [0..1]' -x 'Inliers angular distribution histogram bins [0..1]' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_detailed.csv -o ${results_directory}/registered-points-angular-distribution-outliers-distributions -c 24 -t 'Probability distributions for outliers angular distribution [0..1]' -x 'Outliers angular distribution histogram bins [0..1]' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &

# Number of points after several processing stages
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_diagnostics.csv -o ${results_directory}/pointclouds-size-number-points-reference-pointcloud-distributions -c 4 -t 'Probability distributions for number of points in reference pointcloud' -x 'Pointcloud size' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_diagnostics.csv -o ${results_directory}/pointclouds-size-number-points-reference-pointcloud-after-filtering-distributions -c 5 -t 'Probability distributions for number of points in reference pointcloud after filtering' -x 'Pointcloud size' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_diagnostics.csv -o ${results_directory}/pointclouds-size-number-keypoints-reference-pointcloud-distributions -c 6 -t 'Probability distributions for number of keypoints in reference pointcloud' -x 'Pointcloud size' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_diagnostics.csv -o ${results_directory}/pointclouds-size-number-points-ambient-pointcloud-distributions -c 7 -t 'Probability distributions for number of points in ambient pointcloud' -x 'Pointcloud size' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_diagnostics.csv -o ${results_directory}/pointclouds-size-number-points-ambient-pointcloud-after-filtering-distributions -c 8 -t 'Probability distributions for number of points in ambient pointcloud after filtering' -x 'Pointcloud size' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_diagnostics.csv -o ${results_directory}/pointclouds-size-number-points-ambient-pointcloud-used-in-registration-distributions -c 9 -t 'Probability distributions for number of points in ambient pointcloud used in registration' -x 'Pointcloud size' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_diagnostics.csv -o ${results_directory}/pointclouds-size-number-keypoints-ambient-pointcloud-distributions -c 10 -t 'Probability distributions for number of keypoints in ambient pointcloud' -x 'Pointcloud size' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &

# Processing time of the main processing stages
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_times.csv -o ${results_directory}/computation-times-milliseconds-global-time-distributions -c 4 -t 'Probability distributions for global time (milliseconds)' -x 'Computation time histogram bins (milliseconds)' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_times.csv -o ${results_directory}/computation-times-milliseconds-filtering-time-distributions -c 5 -t 'Probability distributions for filtering time (milliseconds)' -x 'Computation time histogram bins (milliseconds)' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_times.csv -o ${results_directory}/computation-times-milliseconds-surface-normal-estimation-time-distributions -c 6 -t 'Probability distributions for surface normal estimation time (milliseconds)' -x 'Computation time histogram bins (milliseconds)' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_times.csv -o ${results_directory}/computation-times-milliseconds-keypoint-selection-time-distributions -c 7 -t 'Probability distributions for keypoint selection time (milliseconds)' -x 'Computation time histogram bins (milliseconds)' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_times.csv -o ${results_directory}/computation-times-milliseconds-initial-pose-estimation-time-distributions -c 8 -t 'Probability distributions for initial pose estimation time (milliseconds)' -x 'Computation time histogram bins (milliseconds)' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_times.csv -o ${results_directory}/computation-times-milliseconds-pointcloud-registration-time-distributions -c 9 -t 'Probability distributions for pointcloud registration time (milliseconds)' -x 'Computation time histogram bins (milliseconds)' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_times.csv -o ${results_directory}/computation-times-milliseconds-outlier-detection-time-distributions -c 10 -t 'Probability distributions for outlier detection time (milliseconds)' -x 'Computation time histogram bins (milliseconds)' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_times.csv -o ${results_directory}/computation-times-milliseconds-registered-points-angular-distribution-analysis-time-distributions -c 11 -t 'Probability distributions for registered points angular distribution analysis time (milliseconds)' -x 'Computation time histogram bins (milliseconds)' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_times.csv -o ${results_directory}/computation-times-milliseconds-transformation-validators-time-distributions -c 12 -t 'Probability distributions for transformation validators time (milliseconds)' -x 'Computation time histogram bins (milliseconds)' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &


wait


probability_distributions_csv_final="${results_directory}/probability_distributions.csv"
echo -n "" > ${probability_distributions_csv_final}
echo "%file,norm_location,norm_scale,lognorm_location,lognorm_scale,lognorm_shape,genextreme_location,genextreme_scale,genextreme_shape" > ${probability_distributions_csv_final}
sort ${probability_distributions_csv} >> ${probability_distributions_csv_final}

rm -f ${probability_distributions_csv}


echo "\n############################################################################################################################################################"
echo "##### Finished generating results for ${results_directory}"
echo "############################################################################################################################################################\n"
