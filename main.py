import subprocess
import sys

def run_script(script_name):
    """Run a Python script and handle errors."""
    try:
        result = subprocess.run([sys.executable, script_name], check=True)
        print(f"Successfully ran {script_name}")
    except subprocess.CalledProcessError as e:
        print(f"Error while running {script_name}: {e}")
        sys.exit(1)
    except FileNotFoundError:
        print(f"Script {script_name} not found.")
        sys.exit(1)

if __name__ == "__main__":
    # Run scan.py
    run_script("detection.py")

    # Run detection.py
    run_script("detection.py")