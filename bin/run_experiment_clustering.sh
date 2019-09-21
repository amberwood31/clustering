INPUT=$1 # folder path
FILENAME=$2
SEED=$3
INLIER_QUANTITY=$4
OUTLIER_QUANTITY=$5
OUTLIER_THRESHOLD=$6



cp "$INPUT""$FILENAME".g2o_seed_"$SEED"_sorted.g2o sorted.g2o
cp "$INPUT""$FILENAME".g2o_seed_"$SEED"_del0.g2o input.g2o
/home/amber/stew/slam++/bin/slam_incre_clustering_lib -i sorted.g2o >&1 | tee /home/amber/stew/slam++/bin/clustering_output.txt
./analyze_clustering.sh $INLIER_QUANTITY $OUTLIER_QUANTITY $OUTLIER_THRESHOLD >&1 | tee /home/amber/stew/slam++/bin/clustering_analysis.txt

