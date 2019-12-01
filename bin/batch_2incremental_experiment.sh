INPUT=$1 # folder path
FILENAME=$2
OUTLIER_STRATETY=$3
END=$4
INLIER_QUANTITY=$5
OUTLIER_QUANTITY=$6
#OUTLIER_THRESHOLD=$7

for ((SEED=1;SEED<=END;SEED++))
do
    echo $SEED
    if [ $FILENAME == "csail" ]; then
        echo "csail"
        cp "$INPUT""$FILENAME".g2o_unique.g2o_seed_"$SEED"_sorted.g2o sorted.g2o
        cp "$INPUT""$FILENAME".g2o_unique.g2o_seed_"$SEED"_del0.g2o input.g2o

    else
	cp "$INPUT""$FILENAME".g2o_seed_"$SEED"_sorted.g2o sorted.g2o
	cp "$INPUT""$FILENAME".g2o_seed_"$SEED"_del0.g2o input.g2o
    fi


	/home/amber/stew/slam++/bin/slam_incre_clustering -i sorted.g2o -cs 10 >&1 | tee /home/amber/stew/slam++/bin/clustering_output_"$SEED".txt
	python examine_clustering_results.py input.g2o clustering_results.txt rejected_loops.txt $INLIER_QUANTITY $OUTLIER_QUANTITY >&1 | tee /home/amber/stew/slam++/bin/clustering_analysis_"$SEED".txt

	mv full_analysis.txt full_analysis_"$SEED".txt
        mv input.g2o input_"$SEED".g2o
        mv clustering_results.txt clustering_results_"$SEED".txt
	mv rejected_loops.txt rejected_loops_"$SEED".txt

done


cd ../../test_backend/
mkdir slampp_clustering_"$FILENAME"_"$OUTLIER_STRATETY"
cd ../slam++/bin/
cp clustering_output_*.txt clustering_analysis_*.txt full_analysis_*.txt input_*.g2o clustering_results_*.txt examine_clustering_results.py rejected_loops_*.txt ../../test_backend/slampp_clustering_"$FILENAME"_"$OUTLIER_STRATETY"

