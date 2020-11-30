# Raspberry-Pi-Car-with-AprilTag-Detection
A raspberry pi car that can detect its next direction of movement by scanning the apriltag.

Firstly you have to install the apriltag which is based on https://github.com/swatbotics/apriltag.
Then connect your camera and raspberry pi car properly then can run the finalver.py.

Some description of the part of the code:
For the scanner():
 * Tagid为从图形码中识别的编号（初设为2是因为2是直走的编号）
 * 使用opencv里的拍摄功能拍照并储存成image.jpg
 * 利用apriltag里的detector来读取image.jpg并识别编号
 * 我们把二维码数据存到result里面，如果读到了二维码，result就是一个二维数组长度不为0,这时我们读取result[0][1]为二维码的id,根据id的不同让小车做出不同的动作
 * 两个continue语句是为了避免小车失控且多次扫码导致错误输出，故两次重复扫到左右转时让车继续直行
 * Tagid=0时为右转,Tagid=1时为左转,Tagid=2时为直走,Tagid=3时为停止
 * L_control.ideal_speed和R_control.ideal_speed为pid用来更新轮子速度的一个变量
 * 之所以加入time.sleep()是因为摄像头扫描图形码的时间无法确定，故为了避免小车跑太远且给予充足时间扫码才加了time.sleep()

![alt text](https://github.com/heavenluv/Raspberry-Pi-Car-with-AprilTag-Detection/blob/main/raspberrypicar.jpg)
