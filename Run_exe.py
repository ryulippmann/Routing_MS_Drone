import subprocess

# Path to the Routing_mothership_drone executable
executable_path = './x64/Release/Routing_MS_Drone'

def call_executable():
    try:
        # Call the executable
        result = subprocess.run([executable_path], check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        # Print the output and errors, if any
        print(result.stdout.decode())

        if result.stderr:
            print(result.stderr.decode())

    except subprocess.CalledProcessError as e:
        print(f"Error occurred: {e}")

def main():
    for i in range(10):
        print(f"Running iteration {i+1}")
        call_executable()

if __name__ == '__main__':
    main()
