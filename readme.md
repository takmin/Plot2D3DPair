# Plot2d3DPair

For calibrate pose and position between 3D sensor and 2D camera, this tool helps to plot point pair between 3D point cloud and 2D image.

The point pair can be plotted in the following steps:
1. Start the program
2. At "Point Cloud File:" prompt, input pcd or ply file path. Then point cloud viewer is opened.
3. At "Image File:" prompt, input image file path (jpg, png, bmp, etc). Then image viewer is opened.
4. At point cloud viewer, choose points with "shift + left click".
5. At image viewer, choose correspoinding points with left click.
6. When you finish to choose corresponding points, then push "q" button at each windows to close.
7. At "Quit? (1:Yes, 0:No):" prompt, input "0" if you have other point cloud / image pairs, or input "1" otherwise.
8. If you input "0", process repeats from step2.

You can see help with button "h" in each point cloud and image viewer.
