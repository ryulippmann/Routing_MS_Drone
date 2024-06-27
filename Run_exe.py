import subprocess
import gc

# Path to the Routing_mothership_drone executable
# specify which .exe file to run
executable_path = './x64/Release/1.5E5@99.9925_Routing_MS_Drone'

def call_executable():
    try:
        # Call the executable
        # result =
        subprocess.run([executable_path], check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        # # Print the output and errors, if any
        # print(result.stdout.decode())

        # if result.stderr:
        #     print(result.stderr.decode())
        # return result
        return
    except subprocess.CalledProcessError as e:
        print(f"Error occurred: {e}")
        return None

def main():
    for i in range(10):
        print(f"Running iteration {i}")
        result = call_executable()
        # clear memory!
        # if result is not None:
        #     del result
        # gc.collect()

if __name__ == '__main__':
    main()
