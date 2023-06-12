# Verbaliser
This is a ROS2 package for simply integrating voice detection and AI chat responses to a robotic system.

### Overview
There are included, three scripts:
1. audio_collector.py
2. openai_chat.py
3. speaker.py

### Pipeline
The standard pipeline works as follows:
1. a trigger is sent by whatever system you wish to act as your initiation (be it on a response to a physical button, reciept of a specific message, or by some other stimulus), this is to be sent to the topic `/audio_collector/trigger`
2. the audio collector activates the microphone to begin listening (by default, it will try listning for five seconds before exiting early, and once it begins detecting sound, it will listen for 10 seconds total before cutting it short).
3. the audio will be sent to the google cloud platform to parse the audio to text
4. the resulting text will be sent out on a topic `/openai/text`
5. this is recieved by the openai_chat script, which sends the new input, along with the chat so far, to the openai server to start the processing
6. on reciept of a response, it is formatted into a single string and published on the out topic `/speaker/input`
7. the speaker.py script then processes this text and sends it to console under the espeak package for it to be output verbally
8. on completion of this response, the speaker script publishes a follow up trigger to the audio collector for the cycle to repeat
9. the cycle ends when the audio collector is unable to parse an output from the microphone due to silence

### Instalation
#### Installing from source (assuming humble is already installed)
1. create and cd into a new colcon directory: `mkdir -p ~/chat_ws/src; cd ~/chat_ws/src`
2. clone the repository into thie directory using: `git clone https://github.com/Iranaphor/verbaliser.git`
3. create a bash management file in your directory: `touch ~/chat_ws/src/runcom.sh`
4. open this newly created file and paste the following script into it: `gedit ~/chat_ws/src/runcom.sh`
```sh
source /opt/ros/humble/setup.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/humble/
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
[ -f $CHAT_DIR/install/setup.bash ] && source $CHAT_DIR/install/setup.bash

# Define maintanence functions
function chat_rebuild() {
  [ -d $CHAT_DIR/build/ ] && rm -r $CHAT_DIR/build/;
  [ -d $CHAT_DIR/install/ ] && rm -r $CHAT_DIR/install/;
  [ -d $CHAT_DIR/log/ ] && rm -r $CHAT_DIR/log/;
  export AMENT_PREFIX_PATH=""
  export CMAKE_PREFIX_PATH=""
  source /opt/ros/humble/setup.bash

  # Install dependencies
  cd $CHAT_DIR;
  rosdep install -i --from-path src --rosdistro humble -y --skip-keys="$@";

  # Build packages
  colcon build --symlink-install --packages-skip $@;

  # Reset environment
  cd $OLDPWD;
  source $CHAT_DIR/install/setup.bash;
}

export VERBALISER_ROS=$CHAT_DIR/src/verbaliser
export VERBALISER_TMULE=$VERBALISER_ROS/new_tmule/chat.tmule.yaml
function vm () { tmule -c $VERBALISER_TMULE $1 ; }
```

5. add the following to the end of your `~/.bashrc` file, replacing `chat_ws` with the root directory of your workspace:
```sh
export OPENAI_API_KEY="insert-api-key-here"
export CHAT_DIR=$HOME/chat_ws
source $CHAT_DIR/src/runcom.sh
```
6. close the terminal you have open and create a new one
7. perform a build of your workspace using `chat_rebuild`
8. once it has built, close and open a new terminal
9. to initiate the system, use the command: `vm launch`
10. to close the system, use the command: `vm terminate`











