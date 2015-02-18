#!/bin/sh

##################################################
################### parameters ###################
##################################################

videos_directory=${1:?'Must specify the directory of the videos'}
number_of_images=${2:-10}
sampling_rate=${3:-360}
image_extension=${4:-png}
compression_level=${5:-8}
quality=${6:-31}
videos_extension=${7:-mp4}


echo "############################################################################################################################################################"
echo "##### Generating images for videos in path: ${videos_directory}"
echo "############################################################################################################################################################\n"

cd ${videos_directory}

for file in *.${videos_extension}
do
	images_path=images-${file%.*}
	mkdir -p ${images_path}
	
	if [ ${number_of_images} -gt 0 ]; then
		number_of_frames_in_video=`ffprobe -show_streams ${file} | grep "nb_frames" | cut -d '=' -f 2`
		export sampling_rate=`echo "(${number_of_frames_in_video}/(${number_of_images}-1))-1" | bc`
		echo "##### Number of frames in video: ${number_of_frames_in_video}"
		
		#duration_of_video=`ffprobe -show_streams ${file} | grep "duration" | cut -d '=' -f 2`
		#export sampling_rate=`echo "${number_of_images}/${duration_of_video}" | bc -l`
	fi
	
	echo "##### Video sampling rate: ${sampling_rate}"
	avconv -i ${file} -filter:v "select=not(mod(n\,${sampling_rate}))" -compression_level ${compression_level} -qscale ${quality} ${images_path}/image%01d.${image_extension} &
	#avconv -i ${file} -r 0.0344 -vframes ${number_of_images} -compression_level ${compression_level} -qscale ${quality} ${images_path}/image%02d.${image_extension} &
done


wait

echo "\n############################################################################################################################################################"
echo "##### Finished generating images for path ${videos_directory}"
echo "############################################################################################################################################################\n"
