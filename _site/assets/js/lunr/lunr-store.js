var store = [{
        "title": "Vim Cheatsheet",
        "excerpt":"Plugins ‘VundleVim/Vundle.vim’ ‘NLKNguyen/papercolor-theme’ ‘rip-rip/clang_complete’ (install: see below) ‘godlygeek/tabular’ ‘plasticboy/vim-markdown’ Install Autocomplete clang_complete command description 1. Download https://apt.llvm.org/llvm.sh   2. chmod u+x ~/Downloads/llvm.sh   3. bash -c ~/Downloads/llvm.sh (einige dependencies - output sagt welche - müssen evtl. manuell nachinstalliert werden) installs LLVM + Clang + compiler-rt + polly + LLDB +...","categories": ["Cheatsheet"],
        "tags": ["vim","cheatsheet"],
        "url": "/cheatsheet/cheatsheet-vim/",
        "teaser": "/assets/images/Vim.jpg"
      },{
        "title": "Linux Cheatsheet",
        "excerpt":"Apps: command description htop activity monitor (sieht besser aus als “top”) hardinfo hardware info ffmpeg mp4 to mp3 converter ffmpeg -i foo.mp4 bar.mp3 convert foo.mp4 to bar.mp3 pyTranscriber generates subtitles for .mp3 files via Google Speech Recognition API using Autosub (GUI) goldendict dict for fast lookup (ctrl + c +...","categories": ["Cheatsheet"],
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
        "excerpt":"Navigation Setup Navigation command description ctrl + alt + v toggle vim mode (this custom shortcut must have been configured previously) ctrl + 0 focus file explorer ctrl + 1 focus editor group 1 ctrl + 2 focus editor group 2, usw. ctrl + w close current editor group (after...","categories": ["Cheatsheet"],
        "tags": ["vscode","cheatsheet"],
        "url": "/cheatsheet/cheatsheet-VSCode/",
        "teaser": "/assets/images/mario-question.png"
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
        "excerpt":"Welcher CMD wird per default beim image Start ausgeführt? Welcher CMD wird per default beim image Start ausgeführt? Klicke auf den letzten der layers in der Liste links. Dann erscheint rechts der zugehörige vollständige CMD. Beispiele Run osrf/ros image with GUI support: xhost +local:root docker run -it --rm -e DISPLAY=$DISPLAY...","categories": ["Notes"],
        "tags": ["docker","notes"],
        "url": "/notes/cheatsheet-docker/",
        "teaser": "/assets/images/mario-question.png"
      },{
        "title": "Git Cheatsheet",
        "excerpt":"Terminology remote = remote-repository (e.g. in git push *remote* *branch*) Basics command description git clone –recurse-submodules repo &lt;Ziel directory&gt;   git branch -a show/list all branches (local and remote) git branch -r show/list all branches (only remote) git show-branch -a show/list all branches and commits (local and remote) git show-branch...","categories": ["Cheatsheet"],
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
        "teaser": "/assets//assets/images/images_databases/databases.png"
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
        "excerpt":"python packages and modules source .py files are modules folders (containing modules) are packages importing a package essentially imports the package’s __init__.py file as a module pyenv For Python version management, e.g. if you want to use multiple python versions on the same machine if a project requires an older...","categories": ["Cheatsheet","Python"],
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
        "excerpt":"Install Über apt install installieren (wie hier beschrieben) Dann, wie in binary installation unter “Installing and initializing rosdep” und “Installing the missing dependencies” beschrieben, die restlichen dependencies installieren (ohne 2. funktioniert ROS2 nicht!). Uninstall sudo apt remove ~nros-galactic-* &amp;&amp; sudo apt autoremove Create Package $ mkdir -p ~/dev_ws/src $ cd...","categories": ["Cheatsheet"],
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
        "url": "/lecture_notes/machine_learning/lecture-notes-ML/",
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
        "excerpt":"Neural Networks Perceptrons (Rosenblatt 1962) perceptrons (SLPs) are generalized linear models (“generalized” because of the activation function) BUT: Deep Neural Networks (MLPs) are nonlinear parametric models. more specifically: perceptrons are generalized linear discriminants (because they map the input x directly to a class label t in {-1,+1} [see above: “Linear...","categories": ["Lecture_Notes","Machine_Learning"],
        "tags": ["lecture_notes","ml"],
        "url": "/lecture_notes/machine_learning/lecture-notes-ML-part2/",
        "teaser": "/assets/images/lenet.png"
      },{
        "title": "ROS Cheatsheet",
        "excerpt":"roslaunch roslaunch --ros-args /path/to/launchfile.launch Display command-line arguments for this launch file &lt;launch&gt; &lt;!-- ros_args.launch --&gt; &lt;arg name=\"foo\" default=\"true\" doc=\"I pity the foo'.\"/&gt; &lt;arg name=\"bar\" doc=\"Someone walks into this.\"/&gt; &lt;arg name=\"baz\" default=\"false\"/&gt; &lt;arg name=\"nop\"/&gt; &lt;arg name=\"fix\" value=\"true\"/&gt; &lt;/launch&gt; $&gt; roslaunch --ros-args ros_args.launch Required Arguments: bar: Someone walks into this. nop: undocumented...","categories": ["Cheatsheet"],
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
      }]
