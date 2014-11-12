#!/bin/sh

##################################################
################### parameters ###################
##################################################

results_directory=${1:?'Must specify directory where the results.bag is and in which the results will be saved'}


echo "############################################################################################################################################################"
echo "##### Generating results for path: ${results_directory}"
echo "############################################################################################################################################################\n"


# generate CSVs from rosbag
rosrun robot_localization_tools bag2csv.sh ${results_directory}/results '/dynamic_robot_localization/diagnostics /dynamic_robot_localization/localization_detailed /dynamic_robot_localization/localization_error /dynamic_robot_localization/odometry_error /dynamic_robot_localization/localization_times /rosout'

mkdir -p "${results_directory}/pdf"
mkdir -p "${results_directory}/svg"
mkdir -p "${results_directory}/eps"


echo "\n======================================================================================="
echo "Building path (with arrows) from the ground truth and localization system poses"
rosrun robot_localization_tools path_plotter.py -i ${results_directory}/results_ground_truth_poses.txt+${results_directory}/results_localization_poses.txt -o ${results_directory}/robot_movement_path -p 1 -v 8 -a 0.0025 -c 'g+b' -t 'Robot movement path (green -> ground truth, blue -> localization system)' -s 1 -q 1 -d 0 &
rosrun robot_localization_tools path_plotter.py -i ${results_directory}/results_ground_truth_poses.txt+${results_directory}/results_localization_poses.txt+${results_directory}/results_odometry_poses.txt -o ${results_directory}/robot_movement_path_with_odometry -p 1 -v 8 -a 0.0025 -c 'g+b+r' -t 'Robot movement path (green -> ground truth, blue -> localization system, red -> odometry)' -s 1 -q 1 -d 0 &


echo "\n======================================================================================="
echo "Building graphs of localization system results"

# Localization errors
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_error.csv -o ${results_directory}/translation_error_components_millimeters -x 2 -y '4+5+6' -w 0.5 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Translation error (millimeters)' -l 'Translation error in the x axis+Translation error in the y axis+Translation error in the z axis' -c 'y+g+b' -t 'Translation errors by axis' -r 1 -g 1 -s 1 -q 1 -d 0 &
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_error.csv -o ${results_directory}/translation_error_millimeters -x 2 -y '7' -w 0.5 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Translation error (millimeters)' -l 'Translation error' -c 'b' -t 'Translation error' -r 1 -g 1 -s 1 -q 1 -d 0 &
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_error.csv -o ${results_directory}/rotation_error_axis -x 2 -y '8+9+10' -w 0.5 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Rotation error axis component [0..1]' -l 'Rotation error x axis component+Rotation error y axis component+Rotation error z axis component' -c 'y+g+b' -t 'Rotation error axis' -r 1 -g 1 -s 1 -q 1 -d 0 &
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_error.csv -o ${results_directory}/rotation_error_degrees -x 2 -y '11' -w 0.5 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Rotation error (degrees)' -l 'Rotation error' -c 'b' -t 'Rotation error' -r 1 -g 1 -s 1 -q 1 -d 0 &

# Odometry errors
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_odometry_error.csv -o ${results_directory}/odometry_translation_error_components_millimeters -x 2 -y '4+5+6' -w 0.5 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Translation error (millimeters)' -l 'Translation error in the x axis+Translation error in the y axis+Translation error in the z axis' -c 'y+g+b' -t 'Translation errors by axis' -r 1 -g 1 -s 1 -q 1 -d 0 &
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_odometry_error.csv -o ${results_directory}/odometry_translation_error_millimeters -x 2 -y '7' -w 0.5 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Translation error (millimeters)' -l 'Translation error' -c 'b' -t 'Translation error' -r 1 -g 1 -s 1 -q 1 -d 0 &
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_odometry_error.csv -o ${results_directory}/odometry_rotation_error_axis -x 2 -y '8+9+10' -w 0.5 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Rotation error axis component [0..1]' -l 'Rotation error x axis component+Rotation error y axis component+Rotation error z axis component' -c 'y+g+b' -t 'Rotation error axis' -r 1 -g 1 -s 1 -q 1 -d 0 &
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_odometry_error.csv -o ${results_directory}/odometry_rotation_error_degrees -x 2 -y '11' -w 0.5 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Rotation error (degrees)' -l 'Rotation error' -c 'b' -t 'Rotation error' -r 1 -g 1 -s 1 -q 1 -d 0 &

# Localization corrections
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_detailed.csv -o ${results_directory}/translation_corrections_components_millimeters -x 2 -y '11+12+13' -w 0.5 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Translation correction (millimeters)' -l 'Translation correction in the x axis+Translation correction in the y axis+Translation correction in the z axis' -c 'y+g+b' -t 'Translation corrections by axis' -r 1 -g 1 -s 1 -q 1 -d 0 &
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_detailed.csv -o ${results_directory}/translation_correction_millimeters -x 2 -y '14' -w 0.5 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Translation correction (millimeters)' -l 'Translation correction' -c 'b' -t 'Translation correction' -r 1 -g 1 -s 1 -q 1 -d 0 &
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_detailed.csv -o ${results_directory}/rotation_correction_axis -x 2 -y '15+16+17' -w 0.5 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Rotation correction axis component [0..1]' -l 'Rotation correction x axis component+Rotation correction y axis component+Rotation correction z axis component' -c 'y+g+b' -t 'Rotation correction axis' -r 1 -g 1 -s 1 -q 1 -d 0 &
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_detailed.csv -o ${results_directory}/rotation_correction_degrees -x 2 -y '18' -w 0.5 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Rotation correction (degrees)' -l 'Rotation correction' -c 'b' -t 'Rotation correction' -r 1 -g 1 -s 1 -q 1 -d 0 &

# Registration analysis
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_detailed.csv -o ${results_directory}/outlier_percentage -x 2 -y '19' -w 0.5 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Outlier percentage [0..1]' -l 'Outlier percentage' -c 'b' -t 'Outlier percentage' -r 1 -g 1 -s 1 -q 1 -d 0 &
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_detailed.csv -o ${results_directory}/root_mean_square_error_inliers -x 2 -y '20' -w 0.5 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Root mean square error (millimeters)' -l 'Root mean square error' -c 'b' -t 'Root mean square error of inliers' -r 1 -g 1 -s 1 -q 1 -d 0 &
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_detailed.csv -o ${results_directory}/registered_points -x 2 -y '21+22' -w 0.5 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Number of points' -l 'Number of registered inliers+Number of registered points (inliers and outliers)' -c 'g+b' -t 'Number registered points' -r 1 -g 1 -s 1 -q 1 -d 0 &
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_detailed.csv -o ${results_directory}/registered_points_angular_distribution -x 2 -y '23+24' -w 0.5 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Angular distribution percentage [0..1]' -l 'Inliers angular distribution+Outliers angular distribution' -c 'g+b' -t 'Registered points angular distribution' -r 1 -g 1 -s 1 -q 1 -d 0 &

# Number of points after several processing stages
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_diagnostics.csv -o ${results_directory}/pointclouds_size -x 2 -y '4+5+6+7+8+9+10' -w 0.5 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Pointcloud size' -l 'Number of points in reference pointcloud+Number of points in reference pointcloud after filtering+Number of keypoints in reference pointcloud+Number of points in ambient pointcloud+Number of points in ambient pointcloud after filtering+Number of points in ambient pointcloud used in registration+Number of keypoints in ambient pointcloud' -c '#7a643b+#708381+#f17008+#f17008+#eeb111+#b00007+#16a111' -t 'Pointclouds size in several processing stages' -r 1 -g 0 -s 1 -q 1 -d 0 &

# Processing time of the main processing stages
rosrun robot_localization_tools graph_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_times.csv -o ${results_directory}/computation_times_milliseconds -x 2 -y '4+5+6+7+8+9+10+11+12' -w 0.5 -m 0.000000001 -n 1 -b 'Time of localization update (seconds from beginning of test)' -v 'Computation time (milliseconds)' -l 'Global time+Filtering time+Surface normal estimation time+Keypoint selection time+Initial pose estimation time+Pointcloud registration time+Outlier detection time+Registered points angular distribution analysis time+Transformation validators time' -c '#7a643b+#708381+#f17008+#f17008+#eeb111+#b00007+#16a111+#1869c5+#9426b2' -t 'Computation times' -r 1 -g 1 -s 1 -q 1 -d 0 &



echo "\n======================================================================================="
echo "Fitting probability distributions to localization system results"

probability_distributions_csv="${results_directory}/probability_distributions_temp.csv"
echo -n "" > ${probability_distributions_csv}

probability_distributions_common_configs="-b -1 -n 100 -m -1 -l 11 -a 10 -w 0.25 -g 4 -r 1 -s 1 -q 1 -d 0 -f 0"

# Localization errors
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_error.csv -o ${results_directory}/translation_error_millimeters_distributions -c 7 -t 'Probability distributions for translation error (millimeters)' -x 'Translation error histogram bins (millimeters)' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_error.csv -o ${results_directory}/rotation_error_degrees_distributions -c 11 -t 'Probability distributions for rotation error (degrees)' -x 'Rotation error histogram bins (degrees)' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &

# Odometry errors
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_odometry_error.csv -o ${results_directory}/odometry_translation_error_millimeters_distributions -c 7 -t 'Probability distributions for translation error (millimeters)' -x 'Translation error histogram bins (millimeters)' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_odometry_error.csv -o ${results_directory}/odometry_rotation_error_degrees_distributions -c 11 -t 'Probability distributions for rotation error (degrees)' -x 'Rotation error histogram bins (degrees)' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &

# Localization corrections
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_detailed.csv -o ${results_directory}/translation_correction_millimeters_distributions -c 14 -t 'Probability distributions for translation correction (millimeters)' -x 'Translation correction histogram bins (millimeters)' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_detailed.csv -o ${results_directory}/rotation_correction_degrees_distributions -c 18 -t 'Probability distributions for rotation correction (degrees)' -x 'Rotation correction histogram bins (degrees)' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &

# Registration analysis
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_detailed.csv -o ${results_directory}/outlier_percentage_distributions -c 19 -t 'Probability distributions for outlier percentage [0..1]' -x 'Outlier percentage histogram bins [0..1]' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_detailed.csv -o ${results_directory}/root_mean_square_error_inliers_distributions -c 20 -t 'Probability distributions for inliers root mean square error (millimeters)' -x 'Root mean square error histogram bins (millimeters)' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_detailed.csv -o ${results_directory}/registered_points_angular_distribution_inliers_distributions -c 23 -t 'Probability distributions for inliers angular distribution [0..1]' -x 'Inliers angular distribution histogram bins [0..1]' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_detailed.csv -o ${results_directory}/registered_points_angular_distribution_outliers_distributions -c 24 -t 'Probability distributions for outliers angular distribution [0..1]' -x 'Outliers angular distribution histogram bins [0..1]' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &

# Number of points after several processing stages
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_diagnostics.csv -o ${results_directory}/pointclouds_size_number_points_reference_pointcloud_distributions -c 4 -t 'Probability distributions for number of points in reference pointcloud' -x 'Pointcloud size' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_diagnostics.csv -o ${results_directory}/pointclouds_size_number_points_reference_pointcloud_after_filtering_distributions -c 5 -t 'Probability distributions for number of points in reference pointcloud after filtering' -x 'Pointcloud size' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_diagnostics.csv -o ${results_directory}/pointclouds_size_number_keypoints_reference_pointcloud_distributions -c 6 -t 'Probability distributions for number of keypoints in reference pointcloud' -x 'Pointcloud size' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_diagnostics.csv -o ${results_directory}/pointclouds_size_number_points_ambient_pointcloud_distributions -c 7 -t 'Probability distributions for number of points in ambient pointcloud' -x 'Pointcloud size' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_diagnostics.csv -o ${results_directory}/pointclouds_size_number_points_ambient_pointcloud_after_filtering_distributions -c 8 -t 'Probability distributions for number of points in ambient pointcloud after filtering' -x 'Pointcloud size' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_diagnostics.csv -o ${results_directory}/pointclouds_size_number_points_ambient_pointcloud_used_in_registration_distributions -c 9 -t 'Probability distributions for number of points in ambient pointcloud used in registration' -x 'Pointcloud size' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_diagnostics.csv -o ${results_directory}/pointclouds_size_number_keypoints_ambient_pointcloud_distributions -c 10 -t 'Probability distributions for number of keypoints in ambient pointcloud' -x 'Pointcloud size' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &

# Processing time of the main processing stages
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_times.csv -o ${results_directory}/computation_times_milliseconds_global_time_distributions -c 4 -t 'Probability distributions for global time (milliseconds)' -x 'Computation time histogram bins (milliseconds)' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_times.csv -o ${results_directory}/computation_times_milliseconds_filtering_time_distributions -c 5 -t 'Probability distributions for filtering time (milliseconds)' -x 'Computation time histogram bins (milliseconds)' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_times.csv -o ${results_directory}/computation_times_milliseconds_surface_normal_estimation_time_distributions -c 6 -t 'Probability distributions for surface normal estimation time (milliseconds)' -x 'Computation time histogram bins (milliseconds)' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_times.csv -o ${results_directory}/computation_times_milliseconds_keypoint_selection_time_distributions -c 7 -t 'Probability distributions for keypoint selection time (milliseconds)' -x 'Computation time histogram bins (milliseconds)' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_times.csv -o ${results_directory}/computation_times_milliseconds_initial_pose_estimation_time_distributions -c 8 -t 'Probability distributions for initial pose estimation time (milliseconds)' -x 'Computation time histogram bins (milliseconds)' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_times.csv -o ${results_directory}/computation_times_milliseconds_pointcloud_registration_time_distributions -c 9 -t 'Probability distributions for pointcloud registration time (milliseconds)' -x 'Computation time histogram bins (milliseconds)' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_times.csv -o ${results_directory}/computation_times_milliseconds_outlier_detection_time_distributions -c 10 -t 'Probability distributions for outlier detection time (milliseconds)' -x 'Computation time histogram bins (milliseconds)' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_times.csv -o ${results_directory}/computation_times_milliseconds_registered_points_angular_distribution_analysis_time_distributions -c 11 -t 'Probability distributions for registered points angular distribution analysis time (milliseconds)' -x 'Computation time histogram bins (milliseconds)' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${results_directory}/results__dynamic_robot_localization_localization_times.csv -o ${results_directory}/computation_times_milliseconds_transformation_validators_time_distributions -c 12 -t 'Probability distributions for transformation validators time (milliseconds)' -x 'Computation time histogram bins (milliseconds)' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &


wait


probability_distributions_csv_final="${results_directory}/probability_distributions.csv"
echo -n "" > ${probability_distributions_csv_final}
echo "%file,norm_location,norm_scale,lognorm_location,lognorm_scale,lognorm_shape,genextreme_location,genextreme_scale,genextreme_shape" > ${probability_distributions_csv_final}
sort ${probability_distributions_csv} >> ${probability_distributions_csv_final}

rm -f ${probability_distributions_csv}


echo "\n############################################################################################################################################################"
echo "##### Finished generating results for ${results_directory}"
echo "############################################################################################################################################################\n"
