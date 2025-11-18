from mfrc522 import SimpleMFRC522
import time

print("RC522 test: tap a tag...")
reader = SimpleMFRC522()
try:
	while True:
		uid, text = reader.read()
		print("UID (hex):", format(uid, "X"))
		time.sleep(0.5)
except KeyboardInterrupt:
	print("\nBye")

