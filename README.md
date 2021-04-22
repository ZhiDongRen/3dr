# 3dr
3D surface reconstruction
pointcloud
3d 表面重建 
How to run this project?
requirements: PCL 1.2 and boost
git clone https://github.com/ZhiDongRen/3dr.git
cd 3dr 
mkdir build && cd build 
cmake ..  this step make for sure you are installed necessary PCL modules
make 

usage: ./gp3 <your pcd file> 
usage: ./Bspline <your pcd file> -rn 4 -in 10
usage: ./convex <your pcd file>
good luck!
