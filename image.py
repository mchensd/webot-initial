# Put this file in the same folder as the controller.
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1.axes_rgb import RGBAxes
import numpy as np
import ast

with open('im1.txt', 'r') as f:
    pixels = f.readline()
   
arr = np.array(ast.literal_eval(pixels))

print(len(arr))
print(arr.shape)


R = arr[:,:,0].T
G = arr[:,:,1].T
B = arr[:,:,2].T
#plt.imshow([[7,8,9]])
with open('world_info.txt', 'r') as f:
    pixels2 = f.readline()
   
pixels2 = ast.literal_eval(pixels2)
rgb_dict = {
    'f': [216,183,171],
    'r': [224,68,48],
    'o': [212,140,95],
    'y': [224,214,32],
    'p': [209,137,181],
    'b': [36,149,235],
    '0': [0,0,0]
}
rgb_arr2 = []
for i in range(len(pixels2)):
    row = []
    for j in range(len(pixels2[i])):
        row.append(rgb_dict[pixels2[i][j]])
    rgb_arr2.append(row)
arr2 = np.array(rgb_arr2)

R2 = arr2[:,:,0]
G2 = arr2[:,:,1]
B2 = arr2[:,:,2]

fig = plt.figure()
ax = RGBAxes(fig, [0.1, 0.4, 0.6, 0.8])
ax.imshow_rgb(R,G,B)

ax2 = RGBAxes(fig, [.1,0,.6,.8])
ax2.imshow_rgb(R2,G2,B2)
#ax.imshow_rgb(np.array([[255,255,0,0],[0,0,255,255]]), np.array([[0,255,255,255],[0,255,0,0]]), np.array([[0,0,0,0,],[0,255,0,255]]))
plt.show()

#pixels_out = [(255,0,0)]
#image_out = Image.new()
