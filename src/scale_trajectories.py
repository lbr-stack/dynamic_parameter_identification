#!/usr/bin/python3
import csv
import numpy as np

qs_des_ = []
with open(
        "/home/thy/target_joint_states.csv", newline=""
    ) as csvfile:
        csv_reader = csv.DictReader(csvfile)

        # read csv and fill self.qs_des_
        for row in csv_reader:
            qs_des_.append(
                [float(qi_des) for qi_des in list(row.values())[0:8]]
            )

output_path = "/home/thy/target_joint_states_scale.csv"
keys = ["lbr_A0", "lbr_A1", "lbr_A2", "lbr_A3", "lbr_A4", "lbr_A5", "lbr_A6"]
keys = ["time_stamps"] + keys
# qs_des_ = []
print("qs", qs_des_[1])
with open(output_path, "w") as csvfile:
    csv_writer = csv.DictWriter(csvfile, fieldnames=keys)

    csv_writer.writeheader()
        # for values in values_list:
    for i in range(1,len(qs_des_)):
        for j in range(10):
            value = [(np.asarray(item_1) 
                     + float(j)/10.0*(np.asarray(item)-np.asarray(item_1))).tolist() 
                     for item_1, item in zip(qs_des_[i-1],qs_des_[i])]
            value[0] = value[0]*10.0
            # print(len(value))
            # csv_writer.writerow(
            #         value
            #     )
            csv_writer.writerow({key: value for key, value in zip(keys,value)})