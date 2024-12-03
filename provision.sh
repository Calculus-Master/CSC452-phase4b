sudo apt-get -y update
sudo apt-get -y install gcc
sudo apt-get -y install build-essential
cd /vagrant/usloss
./configure
make install
