ssh -L 5901:localhost:5900 scifruit@128.97.82.56 -p 2222
vncviewer localhost
conda activate arduino_leptonconda activate arduino_lepton

 scp -P 2222 -r scifruit@128.97.82.56:/home/scifruit/Downloads/arduino-lepton/python_scripts/data/capture_20250420_233831 .

 ssh scifruit@192.168.1.10
conda init 
 conda activate arduino_lepton

 cd /home/scifruit/Downloads/arduino-lepton/python_scripts

Command to see what is on the Nvidia Jetson:
 vncviewer 192.168.1.10:0