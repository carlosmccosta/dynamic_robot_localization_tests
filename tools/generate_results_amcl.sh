#!/bin/sh

##################################################
################### parameters ###################
##################################################

results_directory=${1:?'Must specify directory where the results_amcl.bag is and in which the results will be saved'}
extract_cvs_from_bag=${2:-true}

echo "############################################################################################################################################################"
echo "##### Generating results for path: ${results_directory}"
echo "############################################################################################################################################################\n"


# generate CSVs from rosbag
if [ "${extract_cvs_from_bag}" = true ]; then
	mkdir -p "${results_directory}/amcl_bag"
	mv "${results_directory}/results_amcl.bag" "${results_directory}/amcl_bag/results_amcl.bag"
	rosrun robot_localization_tools bag2csv.sh "${results_directory}/amcl_bag/results_amcl" '/dynamic_robot_localization/localization_error /dynamic_robot_localization/odometry_error /rosout /amcl_pose'
	
	mv "${results_directory}/amcl_bag/results_amcl.bag" "${results_directory}/results_amcl.bag"
	mv "${results_directory}/amcl_bag/results_amcl__dynamic_robot_localization_localization_error.csv" "${results_directory}/results__amcl_localization_error.csv"
	mv "${results_directory}/amcl_bag/results_amcl__dynamic_robot_localization_odometry_error.csv" "${results_directory}/results__amcl_odometry_error.csv"
	mv "${results_directory}/amcl_bag/results_amcl__rosout.csv" "${results_directory}/results__rosout_amcl.csv"
	mv "${results_directory}/amcl_bag/results_amcl__amcl_pose.csv" "${results_directory}/results__amcl_pose.csv"
	
	rm -rf "${results_directory}/amcl_bag"
	
	mkdir -p "${results_directory}/pdf"
	mkdir -p "${results_directory}/svg"
	mkdir -p "${results_directory}/eps"
fi



echo "\n======================================================================================="
echo "Building path (with arrows) from the ground truth and localization system poses"

graphs_colors='g+b+#5C3317+r'

path_files="${results_directory}/results_ground_truth_poses.txt+${results_directory}/results_localization_poses.txt+${results_directory}/results_localization_poses_amcl.txt+${results_directory}/results_odometry_poses.txt"
rosrun robot_localization_tools path_plotter.py -i ${path_files} -o ${results_directory}/robot-movement-path-with-odometry-and-amcl -p 1 -v 8 -a 0.005 -c ${graphs_colors} -t 'Robot movement path (green -> ground truth, blue -> localization system, brown -> amcl, red -> odometry)' -s 1 -q 1 -d 0 &


echo "\n======================================================================================="
echo "Building graphs for AMCL results"

graphs_common_configs="-k 0.75 -r 1 -g 1 -s 1 -q 1 -d 0"
path_graphs_common_configs="--reset 1 --grid 1 -k 0.75 -r 10 -g 3 -s 1 -q 1 -d 0"

# Evolution of x, y and z
rosrun robot_localization_tools graph_plotter.py -i ${path_files} -o ${results_directory}/robot-movement-path-position-evolution-x-with-amcl -x '0-0-0-0' -y '1-1-1-1' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'X position (meters)' -l 'Ground truth+Localization system+AMCL+Odometry' -c ${graphs_colors} -t 'Evolution of x position' ${graphs_common_configs} &
rosrun robot_localization_tools graph_plotter.py -i ${path_files} -o ${results_directory}/robot-movement-path-position-evolution-y-with-amcl -x '0-0-0-0' -y '2-2-2-2' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Y position (meters)' -l 'Ground truth+Localization system+AMCL+Odometry' -c ${graphs_colors} -t 'Evolution of y position' ${graphs_common_configs} &
rosrun robot_localization_tools graph_plotter.py -i ${path_files} -o ${results_directory}/robot-movement-path-position-evolution-z-with-amcl -x '0-0-0-0' -y '3-3-3-3' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Z position (meters)' -l 'Ground truth+Localization system+AMCL+Odometry' -c ${graphs_colors} -t 'Evolution of z position' ${graphs_common_configs} &

# Cumulative position evolution
rosrun robot_localization_tools path_velocity_and_acceleration_plotter.py -i ${path_files} -f -1 -o ${results_directory}/robot-movement-path-position-differences-with-amcl -p 1 -x '0-0-0-0' -y '1-1-1-1' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Position differences from consecutive poses (meters)' -l 'Ground truth+Localization system+AMCL+Odometry' -c ${graphs_colors} -t 'Position differences from consecutive poses' ${path_graphs_common_configs} &
rosrun robot_localization_tools path_velocity_and_acceleration_plotter.py -i ${path_files} -f -1 -o ${results_directory}/robot-movement-path-angular-differences-with-amcl -p 0 -x '0-0-0-0' -y '4-4-4-4' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Angular differences from consecutive poses (degrees)' -l 'Ground truth+Localization system+AMCL+Odometry' -c ${graphs_colors} -t 'Angular differences from consecutive poses' ${path_graphs_common_configs} &
rosrun robot_localization_tools path_velocity_and_acceleration_plotter.py -i ${path_files} -f 0 -o ${results_directory}/robot-movement-path-position-cumulative-with-amcl -p 1 -x '0-0-0-0' -y '1-1-1-1' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Cumulative position differences from consecutive poses (meters)' -l 'Ground truth+Localization system+AMCL+Odometry' -c ${graphs_colors} -t 'Cumulative position differences from consecutive poses' ${path_graphs_common_configs} &
rosrun robot_localization_tools path_velocity_and_acceleration_plotter.py -i ${path_files} -f 0 -o ${results_directory}/robot-movement-path-angular-cumulative-with-amcl -p 0 -x '0-0-0-0' -y '4-4-4-4' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Cumulative angular differences from consecutive poses (degrees)' -l 'Ground truth+Localization system+AMCL+Odometry' -c ${graphs_colors} -t 'Cumulative angular differences from consecutive poses' ${path_graphs_common_configs} &

# Velocity
rosrun robot_localization_tools path_velocity_and_acceleration_plotter.py -i ${path_files} -f 1 -o ${results_directory}/robot-movement-path-linear-velocity-with-amcl -p 1 -x '0-0-0-0' -y '1-1-1-1' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Linear velocity (meters / second)' -l 'Ground truth+Localization system+AMCL+Odometry' -c ${graphs_colors} -t 'Estimated linear velocity' ${path_graphs_common_configs} &
rosrun robot_localization_tools path_velocity_and_acceleration_plotter.py -i ${path_files} -f 1 -o ${results_directory}/robot-movement-path-angular-velocity-with-amcl -p 0 -x '0-0-0-0' -y '4-4-4-4' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Angular velocity (degrees / second)' -l 'Ground truth+Localization system+AMCL+Odometry' -c ${graphs_colors} -t 'Estimated angular velocity' ${path_graphs_common_configs} &

# Acceleration
rosrun robot_localization_tools path_velocity_and_acceleration_plotter.py -i ${path_files} -f 2 -o ${results_directory}/robot-movement-path-linear-acceleration-with-amcl -p 1 -x '0-0-0-0' -y '1-1-1-1' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Linear acceleration (meters / second / second)' -l 'Ground truth+Localization system+AMCL+Odometry' -c ${graphs_colors} -t 'Estimated linear acceleration' ${path_graphs_common_configs} &
rosrun robot_localization_tools path_velocity_and_acceleration_plotter.py -i ${path_files} -f 2 -o ${results_directory}/robot-movement-path-angular-acceleration-with-amcl -p 0 -x '0-0-0-0' -y '4-4-4-4' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Angular acceleration (degrees / second / second)' -l 'Ground truth+Localization system+AMCL+Odometry' -c ${graphs_colors} -t 'Estimated angular acceleration' ${path_graphs_common_configs} &

# Localization errors
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__amcl_localization_error.csv -o ${results_directory}/translation-error-components-millimeters-amcl -x 2 -y '4+5+6' -w 0.25 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Translation error (millimeters)' -l 'Translation error in the x axis+Translation error in the y axis+Translation error in the z axis' -c 'y+g+b' -t 'Translation errors by axis' ${graphs_common_configs} &
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__amcl_localization_error.csv -o ${results_directory}/translation-error-millimeters-amcl -x 2 -y '7' -w 0.25 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Translation error (millimeters)' -l 'Translation error' -c 'b' -t 'Translation error' ${graphs_common_configs} &
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__amcl_localization_error.csv -o ${results_directory}/rotation-error-axis-amcl -x 2 -y '8+9+10' -w 0.25 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Rotation error axis component [0..1]' -l 'Rotation error x axis component+Rotation error y axis component+Rotation error z axis component' -c 'y+g+b' -t 'Rotation error axis' ${graphs_common_configs} &
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__amcl_localization_error.csv -o ${results_directory}/rotation-error-degrees-amcl -x 2 -y '11' -w 0.25 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Rotation error (degrees)' -l 'Rotation error' -c 'b' -t 'Rotation error' ${graphs_common_configs} &



echo "\n======================================================================================="
echo "Fitting probability distributions to localization system results"

probability_distributions_csv="${results_directory}/probability_distributions_amcl_temp.csv"
echo -n "" > ${probability_distributions_csv}

probability_distributions_common_configs="-b -1 -n 100 -m -1 -l 11 -a 10 -w 0.25 -g 4 -r 1 -s 1 -q 1 -d 0 -f 0"

# Localization errors
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__amcl_localization_error.csv -o ${results_directory}/translation-error-millimeters-distributions-amcl -c 7 -t 'Probability distributions for translation error (millimeters)' -x 'Translation error histogram bins (millimeters)' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__amcl_localization_error.csv -o ${results_directory}/rotation-error-degrees-distributions-amcl -c 11 -t 'Probability distributions for rotation error (degrees)' -x 'Rotation error histogram bins (degrees)' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &



wait


probability_distributions_csv_final="${results_directory}/probability_distributions_amcl.csv"
echo -n "" > ${probability_distributions_csv_final}
echo "%file,norm_location,norm_scale,lognorm_location,lognorm_scale,lognorm_shape,genextreme_location,genextreme_scale,genextreme_shape" > ${probability_distributions_csv_final}
sort ${probability_distributions_csv} >> ${probability_distributions_csv_final}

rm -f ${probability_distributions_csv}


echo "\n############################################################################################################################################################"
echo "##### Finished generating results for ${results_directory}"
echo "############################################################################################################################################################\n"
