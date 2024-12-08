# install wiringPi
mkdir -p ~/aguarai-dependences
cd ~/aguarai-dependences
sudo apt-get purge wiringpi
hash -r
git clone https://github.com/WiringPi/WiringPi.git
cd WiringPi
./build
gpio -v