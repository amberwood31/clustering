import os


dataset = ['manhattan', 'intel', 'mit'] #['csail']
inliers_quantity = [1952, 256, 20] #[127]
inliers_percentages = [0.5, 0.6, 0.7, 0.8, 0.9]
sample_size = 10

outliers_quantity = []
for i in range(0, len(dataset)):
    outliers_quantity.append([])
    for inlier_percentage in inliers_percentages:
        inlier_n = inliers_quantity[i]
        outliers_quantity[-1].append(round(inlier_n / inlier_percentage - inlier_n))
#print(outliers_quantity)

os.chdir('/home/amber/stew/slam++/bin/')

for i in range(0,len(dataset)):
    for j in range(0, len(inliers_percentages)):
        dataset_name = dataset[i]
        configuration_name = 'random' + str(outliers_quantity[i][j])
        command = './batch_incremental_experiment.sh /home/amber/stew/pose_dataset/' + dataset_name + '_' + configuration_name + '/ ' + dataset_name + ' ' + configuration_name + ' ' + str(sample_size) + ' ' + str(inliers_quantity[i]) + ' ' + str(outliers_quantity[i][j]) + ' ' + str(0.95)
        print(command)
        os.system(command)


