from st3215 import ST3215
import time
servo = ST3215('/dev/ttyACM0')
print("servo: ",servo)
print("Code Runing...")

print("_______________________")
# print("ping:", servo.PingServo(2))
print(servo.ReadPosition(4))
servo.DefineMiddle(4)
print(servo.ReadPosition(4))

# servo.SetMode(5, 0)
# servo.Rotate(5, 0)
# print(servo.WritePosition(5, 1))
# print(servo.CorrectPosition(5, 5))
# print(servo.WritePosition(5, 2048))


print("_______________________")
print("Success")


# from st3215 import ST3215

# servo = ST3215('/dev/ttyACM0')
# ids = servo.ListServos()
# if ids:
#     print("ids:",ids)
#     servo.MoveTo(ids[0], 2048)
