# # 创建窗口
# fig=plt.figure()
# # 在该窗口中创建3d绘图对象
# ax=Axes3D(fig)
# fig.add_axes(ax)
# # 创建点的x和y坐标数组
# x=np.arange(-3,3,0.2)
# y=np.arange(-3,3,0.2)
# # 网格化处理
# x,y=np.meshgrid(x,y)
# # 计算某个点到原点的距离为半径
# r=np.sqrt(x**2+y**2)
# # 以半径为基准，求它的正弦值为点的z轴坐标
# z=np.sin(r)
# # 绘制图像
# ax.plot_surface(x,y,z,rstride=3,cstride=1,cmap="hot")

# # 底部的投影
# ax.contour(x,y,z, zdir = 'z', offset = -1, cmap = plt.get_cmap('rainbow'))
# # 设置z轴的维度，x,y类似
# ax.set_zlim(-2, 2)
# plt.show()
