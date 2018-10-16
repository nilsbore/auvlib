apps
====

This is mostly notes for the author, @nilsbore, on how to use the programs with our available data.
It is the same commands used to generate the results in the paper.

## parse_gsf_data

This is for generating the submaps for the pockmarks dataset.

```
./parse_gsf_data --swaths /home/nbore/Data/ACFR-tas200810pockmarks/PROCESSED_DATA/r20081015_221314_butts_pockmarks_23_overlappinggrids/bpslam20110606/DT20081015_221314_gsf --sounds /home/nbore/Data/ACFR-tas200810pockmarks/PROCESSED_DATA/r20081015_221314_butts_pockmarks_23_overlappinggrids/bpslam20110606/DT20081015_221314_gsf/sound_speed.data --poses /home/nbore/Data/ACFR-tas200810pockmarks/PROCESSED_DATA/r20081015_221314_butts_pockmarks_23_overlappinggrids/bpslam20110606/dr_pose_est.data --file pockmarks_submaps.cereal
```

## parse_distort_navi_data

This is for generating the submaps for the pipeline dataset.

```
./parse_distort_navi_data --folder /home/nbore/Data/ROS_MMT/MEDGAZ/Exports --file medgaz_submaps.cereal
```

## slam_process_ceres
```
./slam_process_ceres --file medgaz_submaps.cereal --output medgaz_submaps_optimized.cereal --norot
```

## filter_process
```
./filter_process --input medgaz_submaps.cereal --result medgaz_submaps_optimized.cereal
```
