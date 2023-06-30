import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches



if __name__ =='__main__':

    width = 2
    height = 1

    as_data = pd.read_csv("result0.csv")
    
    

    fig = plt.figure()
    ax = plt.axes()
    # r = patches.Rectangle(xy=(9, 1.25+3.0), width=width, height=height, color='black')
    # ax.add_patch(r)

    ax.plot(as_data['x'], as_data['y'], color="red", label="lot")
    has_data = pd.read_csv("result2.csv")
    has_data2 = pd.read_csv("result3.csv")
    ax.plot(has_data['x'], has_data['y'], color="green", label="path1")
    ax.plot(has_data2['x'], has_data2['y'], color="blue", label="path2")

    # has_data3 = pd.read_csv("result4.csv")
    # has_data4 = pd.read_csv("result5.csv")
    # ax.plot(has_data3['x'], has_data3['y'], color="red", label="path3")
    # ax.plot(has_data4['x'], has_data4['y'], color="red", label="path4")

    has_data5 = pd.read_csv("result6.csv")
    ax.plot(has_data5['x'], has_data5['y'], color="red", label="final")

    # r3 = patches.Rectangle(xy=(10.2544, -5.35689 + 3.5 - 0.5), width=1.8, height=5.0,angle=-1.59159,color='blue',alpha=0.8)##0.8
    # ax.add_patch(r3)   


    plt.xlim(0,20)
    plt.ylim(-30,30)
    plt.xlabel('x[m]')
    plt.ylabel('y[m]')
    plt.title('Planning')
    plt.legend()
   
    plt.axis("equal")
    # plt.savefig('result.png')
    plt.show()
