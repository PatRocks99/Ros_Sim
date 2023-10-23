import subprocess

def run_bash_script():
    #Path to script so this can be reused
    script_path ="script.sh"

    #Running the script
    process = subprocess.Popen(["bash", script_path])

    #wait for the script to finish running
    process.wait()

if __name__ == "__main__":
    
    #loop to run script several times
    for i in range(150):
        run_bash_script()