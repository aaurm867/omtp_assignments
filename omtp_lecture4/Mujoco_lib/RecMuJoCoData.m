function robot = RecMuJoCoData
sensor=mj_get_sensor;
robot.JntPos = sensor.sensordata(1:7);
robot.JntVel = sensor.sensordata(8:14);
robot.CartPos = sensor.sensordata(15:17);
robot.CartOri = sensor.sensordata(18:21);
robot.FTcp = sensor.sensordata(22:24)*10;
robot.TTcp = sensor.sensordata(25:27);
robot.Gripper = sensor.sensordata(28);
end