#!/bin/bash

INPUT=$1 # folder path
FILENAME=$2
OUTLIER_STRATETY=$3
END=$4
INLIER_QUANTITY=$5
OUTLIER_QUANTITY=$6
OUTLIER_THRESHOLD=$7

for ((SEED=1;SEED<=END;SEED++))
do
    echo $SEED

	cp "$INPUT""$FILENAME".g2o_unique.g2o_seed_"$SEED"_sorted.g2o sorted.g2o
	cp "$INPUT""$FILENAME".g2o_unique.g2o_seed_"$SEED"_del0.g2o input.g2o
	/home/amber/stew/slam++/bin/slam_outlier_rejection_pofc -i sorted.g2o >&1 | tee /home/amber/stew/slam++/bin/clustering_output_"$SEED".txt
	./analyze_clustering.sh $INLIER_QUANTITY $OUTLIER_QUANTITY $OUTLIER_THRESHOLD >&1 | tee /home/amber/stew/slam++/bin/clustering_analysis_"$SEED".txt
	mv output.txt output_"$SEED".txt
	mv full_analysis.txt full_analysis_"$SEED".txt
	#mv combined_score_plot.png combined_score_plot_"$SEED".png
    mv result.tga result_"$SEED".tga
    rm sorted.g2o input.g2o

done

cd ../../test_backend/
mkdir slampp_"$FILENAME"_"$OUTLIER_STRATETY"
cd ../slam++/bin/
cp clustering_output_*.txt clustering_analysis_*.txt output_*.txt full_analysis_*.txt result_* ../../test_backend/slampp_"$FILENAME"_"$OUTLIER_STRATETY"
cp ../../test_backend/analysis_chamber/batch_slampp_plot.sh ../../test_backend/analysis_chamber/plot_g2o_slampp.py ../../test_backend/slampp_"$FILENAME"_"$OUTLIER_STRATETY"
