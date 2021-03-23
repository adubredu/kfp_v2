#! /bin/bash
 
for ((j=1; j<=50; j++))
do	  
	python get_image_data.py $j
	sleep 15
	 
done

 