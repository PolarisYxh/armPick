驱动：roslaunch caster_moma_bringup caster_moma_j2s6s200_bringup.launch
moveit：roslaunch caster_moma_j2s6s200_moveit_config execute.launch
二维码坐标发布：roslaunch caster_moma_app marker_detect.launch
识别抓取：roslaunch caster_moma_app pick_task.launch 
导航：roslaunch caster_navigation navigation.launch 
导航显示：roslaunch caster_viz display.launch type:=navigation
抓取接口action：/pick_task
