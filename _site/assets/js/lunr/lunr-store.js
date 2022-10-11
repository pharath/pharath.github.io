var store = [{
        "title": "Vim Cheatsheet",
        "excerpt":"Plugins ‘VundleVim/Vundle.vim’ ‘NLKNguyen/papercolor-theme’ ‘rip-rip/clang_complete’ (install: see below) ‘godlygeek/tabular’ ‘plasticboy/vim-markdown’ Install Autocomplete clang_complete Download https://apt.llvm.org/llvm.sh chmod u+x ~/Downloads/llvm.sh bash -c ~/Downloads/llvm.sh (einige dependencies - output sagt welche - müssen evtl. manuell nachinstalliert werden) installs LLVM + Clang + compiler-rt + polly + LLDB + LLD + libFuzzer + libc++ + libc++abi...","categories": ["Cheatsheet"],
        "tags": ["vim","cheatsheet"],
        "url": "/cheatsheet/cheatsheet-vim/",
        "teaser": "/assets/images/Vim.png"
      },{
        "title": "Linux Cheatsheet",
        "excerpt":"Apps command description htop activity monitor (sieht besser aus als “top”) hardinfo hardware info ffmpeg mp4 to mp3 converter ffmpeg -i foo.mp4 bar.mp3 convert foo.mp4 to bar.mp3 ffmpeg -i source.mp4 -ss 00:00:00 -t 00:00:00 -vcodec copy -acodec copy outsplice.mp4 crop source.mp4 from start time -ss to time -t pyTranscriber generates...","categories": ["Cheatsheet"],
        "tags": ["linux","cheatsheet"],
        "url": "/cheatsheet/cheatsheet-linux/",
        "teaser": "/assets/images/linux_teaser.jpg"
      },{
        "title": "Notes",
        "excerpt":"docker starten mit ./scripts/start_docker.sh mehrmals compilen mit ./compile.sh source /opt/ros/eloquent/setup.bash (check, ob alles stimmt: ros2 run demo_nodes_cpp talker und in anderem terminal (hier ohne docker) ros2 run demo_nodes_cpp listener) 4.-8. ist optional: sudo rosdep init rosdep update echo $ROS_DISTRO um ros distro rauszufinden Resolve dependencies rosdep install -i --from-path src...","categories": ["Notes"],
        "tags": ["notes"],
        "url": "/notes/notes-Team-Galaxis/",
        "teaser": "/assets/images/mario-question.png"
      },{
        "title": "VS Code Cheatsheet",
        "excerpt":"Setup install extensions: Remote-Containers on Ubuntu: only Dev-Containers C/C++ Doxygen Documentation Generator CMake CMake Tools Python Pylance Jupyter Jupyter Keymap Jupyter Notebook Renderers TabNine Autocompletion vim Markdown PDF for md2pdf conversion set “Toggle Vim Mode” command keybinding “ctrl + alt + v” in menu “Manage” (unten links) -&gt; keyboard shortcuts...","categories": ["Cheatsheet"],
        "tags": ["vscode","cheatsheet"],
        "url": "/cheatsheet/cheatsheet-VSCode/",
        "teaser": "/assets/images/vscode.png"
      },{
        "title": "Firefox Setup",
        "excerpt":"Profiles Profile Directory about:support -&gt; “Profile Directory” -&gt; “Open Directory” Button History source Firefox stores your history and bookmarks together in a database file named places.sqlite which is in your profile folder. Just copy places.sqlite to another profile folder in order to transfer the history of one profile to another...","categories": ["Setup"],
        "tags": ["firefox","setup"],
        "url": "/setup/notes-firefox/",
        "teaser": "/assets/images/mario-question.png"
      },{
        "title": "RTX 3060 Setup",
        "excerpt":"Install pytorch conda install pytorch torchvision torchaudio cudatoolkit=11.1 -c pytorch -c nvidia (official von pytorch.org) Achtung: Skip cudatoolkit=11.1 Download mit ctrl + c, wenn der Download nicht startet und downloade erst Rest. Google den genauen Namen des cudatoolkit package z.B. linux-64/cudatoolkit-11.1.74-h6bb024c_0.tar.bz2 und downloade dieses package manuell von https://anaconda.org/nvidia/cudatoolkit/files. Installiere linux-64/cudatoolkit-11.1.74-h6bb024c_0.tar.bz2...","categories": ["GPU"],
        "tags": ["gpu","hardware","setup"],
        "url": "/gpu/notes-rtx/",
        "teaser": "/assets/images/mario-question.png"
      },{
        "title": "Ubuntu Cheatsheet",
        "excerpt":"General Shortcuts For Bluetooth Create .desktop file (“.desktop” ending required!) with this content [Desktop Entry] Name=open_bluetooth_settings Exec=gnome-control-center bluetooth Terminal=false Type=Application Make sure that your script is executable, like this: sudo chmod +x /path/to/script.sh (if it does not execute, copy it to ~/Desktop/. There right-click on it and select “Allow Launching”.)...","categories": ["Cheatsheet"],
        "tags": ["ubuntu","cheatsheet"],
        "url": "/cheatsheet/cheatsheet-Ubuntu/",
        "teaser": "/assets/images/mario-question.png"
      },{
        "title": "Docker Notes",
        "excerpt":"docker Location on System command description sudo ls /var/lib/docker/overlay2 hier ist der Großteil aller docker image Daten sudo du -sh $(ls /var/lib/docker/) list size of all files and dirs in /var/lib/docker/ X11 Forwarding xhost + enable GUI for docker xhost +local:root enable GUI for docker docker login docker login registry.git.rwth-aachen.de...","categories": ["Notes"],
        "tags": ["docker","notes"],
        "url": "/notes/cheatsheet-docker/",
        "teaser": "/assets/images/mario-question.png"
      },{
        "title": "Git Cheatsheet",
        "excerpt":"Terminology remote = remote-repository (e.g. in git push *remote* *branch*) Basics command description git rm file1.txt remove the file from the Git repository and the filesystem git rm --cached file1.txt remove the file only from the Git repository and not remove it from the filesystem git add -u :/ git...","categories": ["Cheatsheet"],
        "tags": ["git","cheatsheet"],
        "url": "/cheatsheet/cheatsheet-git/",
        "teaser": "/assets/images/Vim.jpg"
      },{
        "title": "Mac Cheatsheet",
        "excerpt":"Wifi settings   cd /System/Library/PrivateFrameworks/Apple80211.framework/Versions/Current/Resources/ ./airport en1 prefs # set wifi connection behaviour (eg. DisconnectOnLogout=NO)   ssh   Achtung: ssh muss auf Mac erst aktiviert werden: unter “System Preferences”/”Sharing” bei “Remote login” Häkchen setzen!                  command       description                       caffeinate -u       prevent the system from sleeping and (-u for) prevent the system from sleeping source          ","categories": ["Cheatsheet"],
        "tags": ["mac","cheatsheet"],
        "url": "/cheatsheet/cheatsheet-mac/",
        "teaser": "/assets/images/mario-question.png"
      },{
        "title": "Tesla AI Day Summary",
        "excerpt":"Elon Musk What we want to show today is that Tesla is much more than an electric car company that we have deep AI activity in hardware on the inference level on the training level I think we’re arguably the leaders in real world AI as it applies to the...","categories": ["self driving"],
        "tags": ["tesla","self driving"],
        "url": "/self%20driving/post-Tesla-AI-day-2021/",
        "teaser": "/assets/images/tesla-ai-day.png"
      },{
        "title": "Stanford Databases Course",
        "excerpt":"Databases 1.1 Introduction What is a DBMS ? A database management system (DBMS) provides efficient, reliable, convenient&nbsp;and safe&nbsp;multi-user&nbsp;storage of&nbsp;and access to&nbsp;massive&nbsp;amounts of persistent&nbsp;data.massive&nbsp;scale: So if you think about the amount of data that is being produced today, database systems are handling terabytes of data, sometimes even terabytes of data every...","categories": ["Summaries"],
        "tags": ["summaries","databases"],
        "url": "/summaries/summary-Databases/",
        "teaser": "/assets/images/images_databases/databases.png"
      },{
        "title": "Bash Cheatsheet",
        "excerpt":"bash scripting Shebang beste Erklärung: askubuntu discussion oder aus Wikipedia: In computing, a shebang is the character sequence consisting of the characters number sign and exclamation mark (#!) at the beginning of a script. When a text file with a shebang is used as if it is an executable in...","categories": ["Cheatsheet"],
        "tags": ["bash","cheatsheet"],
        "url": "/cheatsheet/cheatsheet-bash/",
        "teaser": "/assets/images/mario-question.png"
      },{
        "title": "Doxygen Cheatsheet",
        "excerpt":"General Experiences/Issues Markdown files are merged Note: This happened when I loaded an old Doxyfile in order to generate the documentation. After that Ichanged the destination dir for the doc and the working dir from which doxygen runs (maybe, the issue arised because the src dir and working dir were...","categories": ["Cheatsheet"],
        "tags": ["doxygen","cheatsheet"],
        "url": "/cheatsheet/cheatsheet-doxygen/",
        "teaser": "/assets/images/mario-question.png"
      },{
        "title": "Python Cheatsheet",
        "excerpt":"Installing multiple Python Versions source: sudo add-apt-repository ppa:deadsnakes/ppa sudo apt-get update sudo apt-get install python3.5 sudo apt-get install python3.5-dev It will not overwrite your existing python3.4 which is still symlinked as python3. Instead, to run python3.5, run the command python3.5 (or python3.X for any other version of python). venv using...","categories": ["Cheatsheet","Python"],
        "tags": ["python","conda","cheatsheet"],
        "url": "/cheatsheet/python/cheatsheet-python/",
        "teaser": "/assets/images/mario-question.png"
      },{
        "title": "Python Cheatsheet",
        "excerpt":"Install am besten mit offline installer (created with Qt Installer Framework); automatische installation via script, das installer automatisch “durchklickt” s. here and here; flags für CLI des offline installers here wenn man qt über apt installiert, muss man qtcreator und die qt library manuell verbinden (umständlich) Vorteil: Installation weiterer Komponenten...","categories": ["Cheatsheet"],
        "tags": ["python","conda","cheatsheet"],
        "url": "/cheatsheet/cheatsheet-qt/",
        "teaser": "/assets/images/mario-question.png"
      },{
        "title": "ROS2 Cheatsheet",
        "excerpt":"Install Über apt install installieren (wie hier beschrieben) Dann, wie in binary installation unter “Installing and initializing rosdep” und bei älteren EOL distros geht rosdep install --from-paths /opt/ros/eloquent/share nur, wenn rosdep update --include-eol-distros ausgeführt wurde - “Installing the missing dependencies” beschrieben, die restlichen dependencies installieren (ohne 2. funktioniert ROS2 nicht!)....","categories": ["Cheatsheet"],
        "tags": ["ros2","ros","cheatsheet"],
        "url": "/cheatsheet/cheatsheet-ros2/",
        "teaser": "/assets/images/mario-question.png"
      },{
        "title": "Andrej Karpathy (Tesla): CVPR 2021 Workshop on Autonomous Vehicles",
        "excerpt":"Introduction i would like to talk a bit more about what tesla has been up for the last few months um so first to start uh i guess we are here at the cbpr uh workshop on autonomous driving so i think i’m preaching to the core a little bit...","categories": ["self driving"],
        "tags": ["tesla","self driving"],
        "url": "/self%20driving/Karpathy-CVPR-2021/",
        "teaser": "/assets/images/tesla-ai-day.png"
      },{
        "title": "Andrej Karpathy - AI for Full-Self Driving at Tesla",
        "excerpt":"Outline I’m very excited to be here to tell you a bit more about how we’re scaling machine learning models data algorithms and infrastructure at Tesla and what that looks like so the outline of my talk is here I’d like to first tell you a little bit about Tesla...","categories": ["self driving"],
        "tags": ["tesla","self driving"],
        "url": "/self%20driving/post-Karpathy-5th-Scaled-ML-Conference-2020/",
        "teaser": "/assets/images/tesla-ai-day.png"
      },{
        "title": "How AI Powers Self-Driving Tesla with Elon Musk and Andrej Karpathy",
        "excerpt":"Introduction so pete told you all about the chip that we’ve designed that runs neural networks in the car my team is responsible for training of these neural networks and that includes all of data collection from the fleet neural network training and then some of the deployment onto that...","categories": ["self driving"],
        "tags": ["tesla","self driving"],
        "url": "/self%20driving/post-Tesla-Autonomy-Day-2019/",
        "teaser": "/assets/images/tesla-ai-day.png"
      },{
        "title": "PyTorch at Tesla - Andrej Karpathy, Tesla",
        "excerpt":"Introduction okay hello everyone I am Andre I am the director of AI at Tesla and I’m very excited to be here to tell you a little bit about PI torch and how we use PI tours to train your all networks for the auto pilot now I’m Pierce to...","categories": ["self driving"],
        "tags": ["tesla","self driving"],
        "url": "/self%20driving/post-Tesla-Karpathy-Pytorch/",
        "teaser": "/assets/images/tesla-ai-day.png"
      },{
        "title": "Tools",
        "excerpt":"CFG (control flow graph) generator   python3   staticfg   source: https://github.com/coetaur0/staticfg.git Error fix: https://github.com/coetaur0/staticfg/issues/16#issue-704702759   Installation   python3 -m venv env source env/bin/activate   ACHTUNG: erst pip install wheel !   Usage      s. github repo README   am besten über das example/build_cfg.py script !  ","categories": ["Notes"],
        "tags": ["tools","notes"],
        "url": "/notes/notes-tools/",
        "teaser": "/assets/images/Vim.jpg"
      },{
        "title": "Machine Learning (Part 1)",
        "excerpt":"General Remarks Don’t lose sight of the bigger picture ! Only learn stuff when you need it ! As Feynman said: don’t read everything about a topic before starting to work on it. Think about the problem for yourself, figure out what’s important, then read the literature. This will allow...","categories": ["Lecture_Notes","Machine_Learning"],
        "tags": ["lecture_notes","ml"],
        "url": "/lecture_notes/machine_learning/lecture-notes-ML-part1/",
        "teaser": "/assets/images/lenet.png"
      },{
        "title": "Network Cheatsheet",
        "excerpt":"MacOS networksetup -setairportpower en1 off networksetup -setairportpower en1 on Ifconfig networksetup ( ohne parameter: zeigt alle möglichen parameter ) alias listssids='/System/Library/PrivateFrameworks/Apple80211.framework/Versions/Current/Resources/airport /usr/local/bin/airport' # (Achtung: ‘ und nicht “ als Anführungszeichen benutzen!) # =&gt; diese Zeile in “vim ~/.bash_profile” einfügen # =&gt; “listssids -s” kann dann alle SSIDs abrufen networksetup -setairportnetwork...","categories": ["Cheatsheet"],
        "tags": ["network","cheatsheet"],
        "url": "/cheatsheet/cheatsheet-network/",
        "teaser": "/assets/images/linux_teaser.jpg"
      },{
        "title": "test latex",
        "excerpt":"Algorithm 1   \\[\\LaTeX\\]  Just a sample algorithmn  ","categories": ["Test"],
        "tags": ["test"],
        "url": "/test/blabla/",
        "teaser": "/assets/images/databases.png"
      },{
        "title": "Computational Graphs in PyTorch",
        "excerpt":"PyTorch 101, Part 1: Understanding Graphs, Automatic Differentiation and Autograd In this article, we dive into how PyTorch’s Autograd engine performs automatic differentiation. 3 years ago • 13 min read By Ayoosh Kathuria MathJax.Hub.Config({ tex2jax: { inlineMath: [[”$”, “$”], [”\\(”, “\\)”]], processEscapes: true } }); PyTorch is one of the...","categories": ["Lecture_Notes","Machine_Learning"],
        "tags": ["lecture_notes","ml"],
        "url": "/lecture_notes/machine_learning/notes-computational-graphs/",
        "teaser": "/assets/images/lenet.png"
      },{
        "title": "PyTorch",
        "excerpt":"PyTorch Doc source Modules read A Simple Custom Module “Note that the module itself is callable, and that calling it invokes its forward() function. This name is in reference to the concepts of “forward pass” and “backward pass”, which apply to each module. The “forward pass” is responsible for applying...","categories": ["PyTorch","Machine_Learning"],
        "tags": ["pytorch","ml"],
        "url": "/pytorch/machine_learning/notes-pytorch/",
        "teaser": "/assets/images/lenet.png"
      },{
        "title": "Triaxiale Schwarzschild-Modelle für elliptische Galaxien und ihre Anwendung auf NGC 4365",
        "excerpt":"Abstract In der vorliegenden Arbeit wird eine Implementierung eines triaxialen Schwarzschild-Modells beschrieben, die von R. C. E. van den Bosch et alii (2008, in [11]) entwickelt wurde. Mit diesem Modell lassen sich eine Vielzahl verschiedener elliptischer Galaxien beschreiben. Es gibt Aufschluss über die interne Dynamik der Galaxie und über deren...","categories": ["Bachelor_Thesis"],
        "tags": ["bachelor_thesis"],
        "url": "/bachelor_thesis/link-Bachelor-Thesis/",
        "teaser": "/assets/images/NGC_4365.jpg"
      },{
        "title": "Learn Blockchains by Building One",
        "excerpt":"Postman Settings To interact with the built blockchain.py API over a network via the Postman App: Select Authorization Type “No Auth” ! select “raw” select “JSON” instead of “Text” Set up python environment # create a python environment python3 -m venv env # activate the python environment source env/bin/activate #...","categories": ["Build","Blockchain"],
        "tags": ["build","blockchain"],
        "url": "/build/blockchain/build-blockchain-dvf/",
        "teaser": "/assets/images/Blockchain.jpg"
      },{
        "title": "Computer Vision",
        "excerpt":" ","categories": ["Summaries"],
        "tags": ["summaries","cv"],
        "url": "/summaries/summary-CV/",
        "teaser": "/assets//assets/images/images_databases/databases.png"
      },{
        "title": "Machine Learning (Part 2)",
        "excerpt":"Neural Networks Perceptrons (Rosenblatt 1962) perceptrons (SLPs) are generalized linear models (“generalized” because of the activation function) BUT: Deep Neural Networks (MLPs) are nonlinear models. more specifically: perceptrons are generalized linear discriminants (because they map the input x directly to a class label t in {-1,+1} [see above: “Linear models...","categories": ["Lecture_Notes","Machine_Learning"],
        "tags": ["lecture_notes","ml"],
        "url": "/lecture_notes/machine_learning/lecture-notes-ML-part2/",
        "teaser": "/assets/images/lenet.png"
      },{
        "title": "ROS Cheatsheet",
        "excerpt":"ROS Packages image_view Display the images of an image topic: rosrun image_view image_view image:=/sensor/camera1/image_raw opencv_apps Install sudo apt install ros-noetic-opencv-apps Hough Transform roslaunch opencv_apps hough_lines.launch image:=/sensor/camera1/image_raw # view all arguments of a node in the launch file roslaunch --args hough_lines /opt/ros/noetic/share/opencv_apps/launch/hough_lines.launch # view all arguments of the launch file roslaunch...","categories": ["Cheatsheet"],
        "tags": ["ros","cheatsheet"],
        "url": "/cheatsheet/cheatsheet-ros/",
        "teaser": "/assets/images/mario-question.png"
      },{
        "title": "Machine Learning (Part 3)",
        "excerpt":"CNN Concept Good Overviews Wikipedia Why do CNNs work? Sparse Connectivity as opposed to Full Connectivity Goodfellow_2016: Convolutional networks, however, typically have sparse interactions (also referred to as sparse connectivity or sparse weights). This is accomplished by making the kernel smaller than the input. This means that we need to...","categories": ["Lecture_Notes","Machine_Learning","Computer_Vision"],
        "tags": ["lecture_notes","ml","cv"],
        "url": "/lecture_notes/machine_learning/computer_vision/lecture-notes-ML-part3/",
        "teaser": "/assets/images/lenet.png"
      },{
        "title": "Carla Simulator",
        "excerpt":"Download release list and doc list package contains a precompiled version of the simulator, the Python API module and some scripts to be used as examples. Install Install Client library pip3 install --upgrade pip (because pip3 version 20.3 or higher is required) pip3 install carla==0.9.12, when you use Carla version...","categories": ["Carla","Simulator","Self_Driving"],
        "tags": ["carla","simulator","self_driving"],
        "url": "/carla/simulator/self_driving/notes-Carla/",
        "teaser": "/assets/images/lenet.png"
      },{
        "title": "New Inspirations for Machine Learning Methods",
        "excerpt":"Bengio youtube: I’d like to see more work done that would really be helpful for the program I’ve been talking about is regarding memory because an attention because the the kind of memory that we’ve put in you on that these days I mean like the extended memory neural nets...","categories": ["Notes","Machine_Learning"],
        "tags": ["notes","ml"],
        "url": "/notes/machine_learning/notes-Neuroscience/",
        "teaser": "/assets/images/lenet.png"
      },{
        "title": "Machine Learning (Part 2) [OLD VERSION]",
        "excerpt":"Neural Networks Perceptrons (Rosenblatt 1962) perceptrons are generalized linear models (“generalized” because of the activation function) BUT: Deep Neural Networks are nonlinear parametric models. more specifically: perceptrons are generalized linear discriminants (because they map the input x directly to a class label t in {-1,+1} [see above: “Linear models for...","categories": ["Lecture_Notes","Machine_Learning"],
        "tags": ["lecture_notes","ml"],
        "url": "/lecture_notes/machine_learning/lecture-notes-ML-part2-old/",
        "teaser": "/assets/images/lenet.png"
      },{
        "title": "Tensorflow",
        "excerpt":"Compatibility (Tensorflow, Python version, Compiler, Build tools, cuDNN and CUDA) see List Check the CUDA version (paths might differ slightly depending on the cuda version): cat /usr/local/cuda/version.txt and cuDNN version: grep CUDNN_MAJOR -A 2 /usr/local/cuda/include/cudnn.h Check GPUs Tensorflow 2 tf.config.list_physical_devices('GPU') print(\"Num GPUs Available: \", len(tf.config.list_physical_devices('GPU'))) Porting (Tensorflow 1 to Tensorflow...","categories": ["Tensorflow","Machine_Learning"],
        "tags": ["tensorflow","ml"],
        "url": "/tensorflow/machine_learning/notes-tensorflow/",
        "teaser": "/assets/images/lenet.png"
      },{
        "title": "Python Plots Cheatsheet",
        "excerpt":"imshow()   plt.imshow just finishes drawing a picture instead of printing it. If you want to print the picture, you just need to add plt.show.  ","categories": ["Cheatsheet","Python","Plotting"],
        "tags": ["python","cheatsheet","plotting"],
        "url": "/cheatsheet/python/plotting/cheatsheet-py-plots/",
        "teaser": "/assets/images/mario-question.png"
      },{
        "title": "Notes on CUDA and cuDNN",
        "excerpt":"Currently installed CUDA 11.2 Update 2 cuDNN 8.1.1.33 Install install CUDA normally install cuDNN without sudo add-apt-repository \"deb https://developer.download.nvidia.com/compute/cuda/repos/${OS}/x86_64/ /\", but use sudo dpkg -i cuda-keyring_1.0-1_all.deb instead Nvidia Blog Post! install nvidia-docker2, sonst funktionieren die Container nicht! Troubleshooting 1 `fatal error: cuda_runtime_api.h: No such file or directory` Is /usr/local/cuda a...","categories": ["Notes","CUDA","cuDNN"],
        "tags": ["cuda","cudnn","notes"],
        "url": "/notes/cuda/cudnn/notes-CUDA/",
        "teaser": "/assets/images/mario-question.png"
      },{
        "title": "Notes on Deepstream, TAO Toolkit, NGC CLI",
        "excerpt":"GStreamer framework DeepStream SDK is based on the GStreamer framework. Wikipedia GStreamer is a pipeline-based multimedia framework that links together a wide variety of media processing systems to complete complex workflows. For instance, GStreamer can be used to build a system that reads files in one format, processes them, and...","categories": ["Notes","Deepstream","TAO_Toolkit","NGC_CLI"],
        "tags": ["notes","deepstream","tao_toolkit","ngc_cli"],
        "url": "/notes/deepstream/tao_toolkit/ngc_cli/notes-Deepstream-TAO-NGC/",
        "teaser": "/assets/images/mario-question.png"
      },{
        "title": "Notes on Jetson Xavier AGX",
        "excerpt":"Jetpack check Jetpack version: sudo apt-cache show nvidia-jetpack SDK Manager Install Troubleshooting Unmet dependencies You might want to run 'apt --fix-broken install' to correct these. The following packages have unmet dependencies: gconf-service-backend : Depends: libgconf-2-4 (= 3.2.6-4ubuntu1) but 3.2.6-4.1 is installed gconf2 : Depends: gconf-service (= 3.2.6-4.1) libgconf-2-4 : Depends:...","categories": ["Notes","AGX"],
        "tags": ["agx","notes"],
        "url": "/notes/agx/notes-AGX/",
        "teaser": "/assets/images/mario-question.png"
      },{
        "title": "Notes on Computer Vision Coding",
        "excerpt":"pickle use the following code to inspect and view .pickle or .p datasets import pickle import matplotlib.pyplot as plt with open('train.p', 'rb') as data: # \"im\" is a dictionary! Get list of keys via im.keys() method. im = pickle.load(data) # without print there is no output in the terminal! #print(im[\"features\"])...","categories": ["Notes","Computer_Vision","OpenCV"],
        "tags": ["notes","computer_vision","opencv","cv2"],
        "url": "/notes/computer_vision/opencv/notes-cv-coding/",
        "teaser": "/assets/images/mario-question.png"
      },{
        "title": "TensorRT",
        "excerpt":"Install WARNING: apt upgrade maybe upgrades libcudnn8 which removes all tensorrt libs! apt-mark hold all libcudnn8 packages before installation! Debian installation, else no samples, no trtexec, … etc. compile trtexec in samples and put export PATH=/usr/src/tensorrt/bin${PATH:+:${PATH}} in .bashrc Install using pip wheel file Check installation import tensorrt as trt print(trt.__version__)...","categories": ["TensorRT","Machine_Learning"],
        "tags": ["tensorrt","ml"],
        "url": "/tensorrt/machine_learning/notes-tensorrt/",
        "teaser": "/assets/images/lenet.png"
      },{
        "title": "Notes on YOLO Object Detection",
        "excerpt":"Anchors      AlexeyAB explanation  ","categories": ["Notes","Computer_Vision","Yolo"],
        "tags": ["notes","computer_vision","yolo"],
        "url": "/notes/computer_vision/yolo/notes-yolo/",
        "teaser": "/assets/images/mario-question.png"
      },{
        "title": "Data Science Basics",
        "excerpt":"Interpreting a PR Curve It is desired that the algorithm should have both high precision, and high recall. However, most machine learning algorithms often involve a trade-off between the two. A good PR curve has greater AUC (area under curve). source Relation to ROC Curve In certain applications (e.g. searching...","categories": ["Notes","Data_Science"],
        "tags": ["notes","data_science"],
        "url": "/notes/data_science/notes-DS/",
        "teaser": "/assets/images/lenet.png"
      },{
        "title": "Notes on Building Code",
        "excerpt":"configure Script      purpose of configure scripts   check “failed” warnings  ","categories": ["Notes","Build"],
        "tags": ["build","notes"],
        "url": "/notes/build/notes-build/",
        "teaser": "/assets/images/mario-question.png"
      },{
        "title": "Embedded Systems",
        "excerpt":"Microcontroller µC $\\approx$ low-end microprocessor + memory + I/O + additional peripherals more general than ASIPs and SoCs Pentium Processor FDIV Bug FDIV Bug deutsch see history of microprocessors modern µCs nowadays may have more than one microprocessor core Structure of a microcontroller Clock clocks that use an electronic oscillator...","categories": ["EmSys","Notes"],
        "tags": ["emsys","notes"],
        "url": "/emsys/notes/notes-EmSys/",
        "teaser": "/assets/images/linux_teaser.jpg"
      },{
        "title": "WSL2 Cheatsheet",
        "excerpt":"Setup Windows 10 Select Start &gt; Settings &gt; Time &amp; language &gt; Language &amp; region. Choose a language from the Windows display language menu, or, next to Preferred languages, select Add a language to install the one you want if it isn’t listed. press Alt + Shift to change keyboard...","categories": ["Cheatsheet"],
        "tags": ["wsl2","cheatsheet"],
        "url": "/cheatsheet/cheatsheet-wsl2/",
        "teaser": "/assets/images/Vim.jpg"
      },{
        "title": "DatKom Notes",
        "excerpt":"Concepts Communication services, layer models, protocols Physical basics of transmission Error handling and medium access Internet Protocol (IP) and Routing: Connecting remote hosts Transmission Control Protocol (TCP): Connecting applications Security: Cryptographic primitives, IPsec, SSL/TLS From [source]: Client/Server- und Peer-to-Peer-Systeme OSI-Referenzmodell und TCP/IP-Referenzmodell Übertragungsmedien und Signaldarstellung Fehlerbehandlung, Flusssteuerung und Medienzugriff Lokale...","categories": ["Notes"],
        "tags": ["datkom","notes"],
        "url": "/notes/notes-DatKom/",
        "teaser": "/assets/images/C_logo.png"
      },{
        "title": "Operating Systems Notes",
        "excerpt":"Concepts Unix shell and programming language C Process management: processes, threads, inter-process communication CPU scheduling Process synchronization, deadlocks Memory management: virtual memory, segmentation, paging, fragmentation File system and I/O system Communication subsystem and sockets Aufgaben und Struktur von Betriebssystemen Das Betriebssystem Unix Systemaufrufe und Shellprogrammierung Einführung in die Programmiersprache C...","categories": ["Notes"],
        "tags": ["os","notes"],
        "url": "/notes/notes-OS/",
        "teaser": "/assets/images/C_logo.png"
      },{
        "title": "C Notes",
        "excerpt":"Concepts RWTH Grundgebiete der Informatik 1 Gegenstand der Vorlesung ist die Einführung in Programmiertechniken, Datenstrukturen und Algorithmen anhand von C. Grundlegende Programmelemente: Skalare und zusammengesetzte Datentypen, Anweisungen, Kontrollfluß, Funktionen, Klassen, C-Programmstruktur und Programmierumgebung; Programmanalyse: Wachstumsordnungen, Komplexitätsklassen, best/worst case Analyse; Lineare Datenstrukturen: Listen, Stacks, Queues, Iteration und Rekursion; Nichtlineare Datenstrukturen und...","categories": ["Notes"],
        "tags": ["c","notes"],
        "url": "/notes/notes-c/",
        "teaser": "/assets/images/C_logo.png"
      },{
        "title": "Software Engineering Notes",
        "excerpt":"Advanced Software Engineering (in Mechanical Engineering) The aim of the course is to explain students for what purposes, under which conditions and with which consequences computer systems are used for the solution of problems related to Mechanical Engineering. Within the first part of the course the steps from problem description...","categories": ["Notes"],
        "tags": ["software_engineering","notes"],
        "url": "/notes/notes-SE/",
        "teaser": "/assets/images/C_logo.png"
      },{
        "title": "Android Cheatsheet",
        "excerpt":"File Transfer   Android 9   To Ubuntu      TODO  ","categories": ["Cheatsheet"],
        "tags": ["android","cheatsheet"],
        "url": "/cheatsheet/cheatsheet-android/",
        "teaser": "/assets/images/C_logo.png"
      },{
        "title": "C++ Notes",
        "excerpt":"DEFINITIONS areas of memory stack - Local variables, function parameters code space - Code global namespace - global variables registers - used for internal housekeeping functions, such as keeping track of the top of the stack and the instruction pointer free store = heap - Just about all of the...","categories": ["Notes"],
        "tags": ["c++","notes"],
        "url": "/notes/notes-cpp/",
        "teaser": "/assets/images/Cpp_logo.png"
      },{
        "title": "HTML Notes",
        "excerpt":"Jekyll Jekyll is written in Ruby. Ruby 101 explains: Gems Gems are code you can include in Ruby projects Ruby gems search Ruby gems list Jekyll Jekyll is a gem. Many Jekyll plugins are also gems, including jekyll-feed, jekyll-seo-tag and jekyll-archives. Gemfile a list of gems used by your site....","categories": ["Notes"],
        "tags": ["html","notes"],
        "url": "/notes/notes-html/",
        "teaser": "/assets/images/mario-question-block.jpeg"
      }]
