# WIP: This is very experimental and probably won't work the first time.
xcode-select --install
brew update >> macsetup.log
brew upgrade >> macsetup.log
brew install cmake >> macsetup.log
brew tap ros/deps >> macsetup.log
brew tap osrf/simulation >> macsetup.log
brew tap homebrew/versions >> macsetup.log
brew tap homebrew/science >> macsetup.log
brew install opencv >> macsetup.log
brew link opencv >> macsetup.log
brew install qt4 >> macsetup.log
brew link qt4 >> macsetup.log
brew unlink qt5 >> macsetup.log
brew link --overwrite qt >> macsetup.log
brew install vtk >> macsetup.log
brew link vtk >> macsetup.log
brew install fltk --devel >> macsetup.log
export PATH=/usr/local/bin:$PATH
source ~/.bashrc
mkdir -p ~/Library/Python/2.7/lib/python/site-packages >> macsetup.log
echo "$(brew --prefix)/lib/python2.7/site-packages" >> ~/Library/Python/2.7/lib/python/site-packages/homebrew.pth
sudo -H pip install -U wstool rosdep rosinstall rosinstall_generator rospkg catkin-pkg Distribute sphinx
sudo rosdep init >> macsetup.log
rosdep update >> macsetup.log
mkdir -p ~/programs/ros_catkin_ws
cd ~/programs/ros_catkin_ws/
rosinstall_generator desktop_full --rosdistro jade --deps --wet-only --tar > jade-desktop-full-wet.rosinstall
wstool init -j8 src jade-desktop-full-wet.rosinstall
rosdep install --from-paths src --ignore-src --rosdistro jade -y
./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space ~/programs/ros_catkin_ws/install_isolated
