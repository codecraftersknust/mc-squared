import time
import subprocess
import RPi.GPIO as GPIO

BUTTON_PIN = 16  # Connect button between GPIO 16 and GND

GPIO.setmode(GPIO.BCM)
# Use internal pull-up resistor so button reads LOW when pressed
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

print("=========================================")
print("🤖 MC-SQUARED LAUNCHER: Ready for track input...")
print("Press button to cycle modes:")
print("  1 Press  -> Round 1 (Left Follow)")
print("  2 Presses -> Round 1 (Right Follow)")
print("  3 Presses -> Round 2 (Left Follow)")
print("  4 Presses -> Round 2 (Right Follow)")
print("=========================================")

press_count = 0
last_press_time = 0
listening_window = False

try:
    while True:
        button_state = GPIO.input(BUTTON_PIN)
        
        # Detect button press (LOW state)
        if button_state == GPIO.LOW:
            current_time = time.time()
            # Simple debounce rule
            if current_time - last_press_time > 0.3:
                press_count += 1
                last_press_time = current_time
                listening_window = True
                print(f"🔘 Click registered! Current Count: {press_count}")
            time.sleep(0.1)
            
        # Wait for 2.5 seconds after the first press to gather total clicks
        if listening_window and (time.time() - last_press_time > 2.5):
            print(f"\nExecution window closed. Total clicks = {press_count}")
            
            if press_count == 1:
                print("🚀 Launching: Round 1 Left Wall Follow...")
                subprocess.run(["python3", "round-1-left.py"])
            elif press_count == 2:
                print("🚀 Launching: Round 1 Right Wall Follow...")
                subprocess.run(["python3", "round-1-right.py"])
            elif press_count == 3:
                print("🚀 Launching: Round 2 Program...")
                subprocess.run(["python3", "round-2-left.py"])
            elif press_count == 4:
                print("Launching: Round 2 Right Wall Program...")
                subprocess.run(["python3", "round-2-right.py"])
            else:
                print("⚠️ Invalid number of clicks. Resetting...")
                
            # Reset listening state
            press_count = 0
            listening_window = False
            print("\nReady for new configuration command...\n")
            
        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nLauncher service terminated.")
finally:
    GPIO.cleanup()
