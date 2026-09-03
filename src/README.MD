GPIO Control requires libgpiod

Install with 
sudo apt update
sudo apt install libgpiod-dev -y

Build package

colcon build

Check 

dpkg -L libgpiod-dev | grep libgpiod

