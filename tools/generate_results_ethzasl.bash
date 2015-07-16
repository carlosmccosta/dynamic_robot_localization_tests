#!/usr/bin/env bash

##################################################
################### parameters ###################
##################################################

results_directory=${1:?'Must specify directory where the results.bag is and in which the results will be saved'}
extract_cvs_from_bag=${2:-true}

echo "############################################################################################################################################################"
echo "##### Generating results for path: ${results_directory}"
echo "############################################################################################################################################################"
echo -e "\n"


# generate CSVs from rosbag
if [ "${extract_cvs_from_bag}" = true ]; then
	rosrun robot_localization_tools bag2csv.bash ${results_directory}/results '/ethzasl_icp_mapper/localization_poses /ethzasl_icp_mapper/groundtruth_poses /ethzasl_icp_mapper/localization_error /rosout'
fi

mkdir -p "${results_directory}/pdf"
mkdir -p "${results_directory}/svg"
mkdir -p "${results_directory}/eps"



echo -e "\n"
echo "======================================================================================="
echo "Building path (with arrows) from the ground truth and localization system poses"


path_files="${results_directory}/results_ground_truth_poses.txt+${results_directory}/results_localization_poses.txt"
rosrun robot_localization_tools path_plotter_3d.py -i ${path_files} -o ${results_directory}/robot-movement-path-3d -p 1 -v 8 -a 0.005 -c 'g+b' -t '' -s 1 -q 1 -d 0 &
rosrun robot_localization_tools path_plotter.py -i ${path_files} -o ${results_directory}/robot-movement-path-xy -p 1 -v 8 -a 0.01 -c 'g+b' -t 'Movement path (green -> ground truth, blue -> localization system)' -s 1 -q 1 -d 0 &
rosrun robot_localization_tools path_plotter.py -i ${path_files} -o ${results_directory}/robot-movement-path-xz -p 1 -e 2 -v 8 -r 2 -a 0.01 -c 'g+b' -m 'z position (meters)' -t 'Movement path (green -> ground truth, blue -> drl localization system)' -s 1 -q 1 -d 0 &

if [ -e "${results_directory}/results_ground_truth_poses_drl.txt" ] && [ -e "${results_directory}/results_localization_poses_drl.txt" ]; then
	path_files_combined="${results_directory}/results_ground_truth_poses_drl.txt+${results_directory}/results_localization_poses_drl.txt+${results_directory}/results_localization_poses.txt"
	rosrun robot_localization_tools path_plotter_3d.py -i ${path_files_combined} -o ${results_directory}/robot-movement-path-combined-3d -p 1 -v 8 -a 0.005 -c 'g+b+r' -t '' -s 1 -q 1 -d 0 &
	rosrun robot_localization_tools path_plotter.py -i ${path_files_combined} -o ${results_directory}/robot-movement-path-combined-xy -p 1 -v 8 -a 0.01 -c 'g+b+r' -t 'Movement path (green -> ground truth, blue -> drl localization system, red -> ethzasl_icp_mapper localization system)' -s 1 -q 1 -d 0 &
	rosrun robot_localization_tools path_plotter.py -i ${path_files_combined} -o ${results_directory}/robot-movement-path-combined-xz -p 1 -e 2 -v 8 -r 2 -a 0.01 -c 'g+b+r' -m 'z position (meters)' -t 'Movement path (green -> ground truth, blue -> drl localization system, red -> ethzasl_icp_mapper localization system)' -s 1 -q 1 -d 0 &
fi

echo -e "\n"
echo "======================================================================================="
echo "Building graphs for localization system results"

graphs_common_configs="-k 0.75 -r 1 -g 1 -s 1 -q 1 -d 0"
path_graphs_common_configs="--reset 1 --grid 1 -k 0.75 -r 10 -g 3 -s 1 -q 1 -d 0"

# Evolution of x, y and z
rosrun robot_localization_tools graph_plotter.py -i ${path_files} -o ${results_directory}/robot-movement-path-position-evolution-x -x '0-0' -y '1-1' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'X position (meters)' -l 'Ground truth x positions+Localization system x positions' -c 'g+b' -t 'Evolution of x position' ${graphs_common_configs} &
rosrun robot_localization_tools graph_plotter.py -i ${path_files} -o ${results_directory}/robot-movement-path-position-evolution-y -x '0-0' -y '2-2' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Y position (meters)' -l 'Ground truth y positions+Localization system y positions' -c 'g+b' -t 'Evolution of y position' ${graphs_common_configs} &
rosrun robot_localization_tools graph_plotter.py -i ${path_files} -o ${results_directory}/robot-movement-path-position-evolution-z -x '0-0' -y '3-3' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Z position (meters)' -l 'Ground truth z positions+Localization system z positions' -c 'g+b' -t 'Evolution of z position' ${graphs_common_configs} &


# Cumulative position evolution
rosrun robot_localization_tools path_velocity_and_acceleration_plotter.py -i ${path_files} -f -1 -o ${results_directory}/robot-movement-path-position-differences -p 1 -x '0-0' -y '1-1' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Position differences from consecutive poses (meters)' -l 'Ground truth+Localization system' -c 'g+b' -t 'Position differences from consecutive poses' ${path_graphs_common_configs} &
rosrun robot_localization_tools path_velocity_and_acceleration_plotter.py -i ${path_files} -f -1 -o ${results_directory}/robot-movement-path-angular-differences -p 0 -x '0-0' -y '4-4' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Angular differences from consecutive poses (degrees)' -l 'Ground truth+Localization system' -c 'g+b' -t 'Angular differences from consecutive poses' ${path_graphs_common_configs} &
rosrun robot_localization_tools path_velocity_and_acceleration_plotter.py -i ${path_files} -f 0 -o ${results_directory}/robot-movement-path-position-cumulative -p 1 -x '0-0' -y '1-1' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Cumulative position differences from consecutive poses (meters)' -l 'Ground truth+Localization system' -c 'g+b' -t 'Cumulative position differences from consecutive poses' ${path_graphs_common_configs} &
rosrun robot_localization_tools path_velocity_and_acceleration_plotter.py -i ${path_files} -f 0 -o ${results_directory}/robot-movement-path-angular-cumulative -p 0 -x '0-0' -y '4-4' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Cumulative angular differences from consecutive poses (degrees)' -l 'Ground truth+Localization system' -c 'g+b' -t 'Cumulative angular differences from consecutive poses' ${path_graphs_common_configs} &


# Velocity
rosrun robot_localization_tools path_velocity_and_acceleration_plotter.py -i ${path_files} -f 1 -o ${results_directory}/robot-movement-path-linear-velocity -p 1 -x '0-0-0' -y '1-1-1' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Linear velocity (meters / second)' -l 'Ground truth+Localization system+Odometry' -c 'g+b+r' -t 'Estimated linear velocity' ${path_graphs_common_configs} &
rosrun robot_localization_tools path_velocity_and_acceleration_plotter.py -i ${path_files} -f 1 -o ${results_directory}/robot-movement-path-angular-velocity -p 0 -x '0-0-0' -y '4-4-4' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Angular velocity (degrees / second)' -l 'Ground truth+Localization system+Odometry' -c 'g+b+r' -t 'Estimated angular velocity' ${path_graphs_common_configs} &


# Acceleration
rosrun robot_localization_tools path_velocity_and_acceleration_plotter.py -i ${path_files} -f 2 -o ${results_directory}/robot-movement-path-linear-acceleration -p 1 -x '0-0' -y '1-1' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Linear acceleration (meters / second / second)' -l 'Ground truth+Localization system' -c 'g+b' -t 'Estimated linear acceleration' ${path_graphs_common_configs} &
rosrun robot_localization_tools path_velocity_and_acceleration_plotter.py -i ${path_files} -f 2 -o ${results_directory}/robot-movement-path-angular-acceleration -p 0 -x '0-0' -y '4-4' -z ' ' -e 2 -w 0.25 -m 1 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Angular acceleration (degrees / second / second)' -l 'Ground truth+Localization system' -c 'g+b' -t 'Estimated angular acceleration' ${path_graphs_common_configs} &


# Localization errors
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__ethzasl_icp_mapper_localization_error.csv -o ${results_directory}/translation-error-components-millimeters -x 2 -y '4+5+6' -w 0.25 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Translation error (millimeters)' -l 'Translation error in the x axis+Translation error in the y axis+Translation error in the z axis' -c 'y+g+b' -t 'Translation errors by axis' ${graphs_common_configs} &
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__ethzasl_icp_mapper_localization_error.csv -o ${results_directory}/translation-error-millimeters -x 2 -y '7' -w 0.25 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Translation error (millimeters)' -l 'Translation error' -c 'b' -t 'Translation error' ${graphs_common_configs} &
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__ethzasl_icp_mapper_localization_error.csv -o ${results_directory}/rotation-error-axis -x 2 -y '8+9+10' -w 0.25 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Rotation error axis component [0..1]' -l 'Rotation error x axis component+Rotation error y axis component+Rotation error z axis component' -c 'y+g+b' -t 'Rotation error axis' ${graphs_common_configs} &
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__ethzasl_icp_mapper_localization_error.csv -o ${results_directory}/rotation-error-degrees -x 2 -y '11' -w 0.25 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Rotation error (degrees)' -l 'Rotation error' -c 'b' -t 'Rotation error' ${graphs_common_configs} &


echo -e "\n"
echo "======================================================================================="
echo "Fitting probability distributions to localization system results"

probability_distributions_csv="${results_directory}/probability_distributions_temp.csv"
echo -n "" > ${probability_distributions_csv}

probability_distributions_common_configs="-b -1 -n 100 -m -1 -l 11 -a 10 -w 0.25 -g 4 -r 1 -s 1 -q 1 -d 0 -f 0"

# Localization errors
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__ethzasl_icp_mapper_localization_error.csv -o ${results_directory}/translation-error-millimeters-distributions -c 7 -t 'Probability distributions for translation error (millimeters)' -x 'Translation error histogram bins (millimeters)' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__ethzasl_icp_mapper_localization_error.csv -o ${results_directory}/rotation-error-degrees-distributions -c 11 -t 'Probability distributions for rotation error (degrees)' -x 'Rotation error histogram bins (degrees)' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &


# Processing time of the main processing stages
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/ConvergenceDuration.csv -o ${results_directory}/computation-times-milliseconds-global-time-distributions -c 0 -e 1000 -t 'Probability distributions for global time (milliseconds)' -x 'Computation time histogram bins (milliseconds)' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &

wait


probability_distributions_csv_final="${results_directory}/probability_distributions.csv"
echo -n "" > ${probability_distributions_csv_final}
echo "%file,norm_location,norm_scale,lognorm_location,lognorm_scale,lognorm_shape,genextreme_location,genextreme_scale,genextreme_shape" > ${probability_distributions_csv_final}
sort ${probability_distributions_csv} >> ${probability_distributions_csv_final}

rm -f ${probability_distributions_csv}


echo -e "\n"
echo "############################################################################################################################################################"
echo "##### Finished generating results for ${results_directory}"
echo "############################################################################################################################################################"
echo -e "\n"
