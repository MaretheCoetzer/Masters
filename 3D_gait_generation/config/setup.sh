#!/bin/sh
# Installs all required dependencies to run the step up
# Application aka KateBush
#------------------------------------------------------------
# GLOBALS
DEFAULT_LIB_DIR="/usr/local/lib"

#------------------------------------------------------------
# Logs said line
logit()
{
	line="[$USER][`date -u +"%Y-%m-%dT%H:%M:%SZ"`] - ${*}"
	echo "$line"
}

#------------------------------------------------------------
# Install python and any pip dependencies alongside it
install_python()
{
	logit "Installing python"
	sudo apt-get install -y python3
	sudo apt -y install python3-pip
	
	# Might need to add it to path WARNING: The script pyomo is installed in '/home/sheldon/.local/bin' which is not on PATH.
	# It did this for all pip deps
	logit "Install pip deps"
	pip3 install numpy
	pip3 install pyomo
	pip3 install sympy
	pip3 install pandas
	pip3 install matplotlib
	pip3 install IPython
	pip3 install pytest
	pip3 install cloudpickle
}

install_ipopt()
{
	#Ipopt bits
	logit "Installing ipopt"
	sudo apt-get install gcc g++ gfortran git patch wget pkg-config liblapack-dev libmetis-dev
	sudo apt-get install coinor-libipopt-dev
	mkdir ~/coinbrew
	cd ~/coinbrew
	wget https://raw.githubusercontent.com/coin-or/coinbrew/master/coinbrew
	chmod +x ./coinbrew
	./coinbrew fetch Ipopt --no-prompt
	
	PARDISO_PATH=${DEFAULT_LIB_DIR}/libpardiso600-GNU800-X86-64.so
	CONFIG_OPTIONS="--with-asl --with-mumps --disable-pardisomkl --with-pardiso=${PARDISO_PATH}"
	# --with-asl --with-mumps --disable-pardisomkl --with-pardiso=/usr/local/lib/libpardiso600-GNU800-X86-64.so
	
	LD_PRELOAD=${DEFAULT_LIB_DIR}/libomp.so ./coinbrew build Ipopt ${CONFIG_OPTIONS}
	sudo cp ~/coinbrew/dist/bin/ipopt /usr/bin
	
	# Install the license to home directory
	cp ../resources/pardiso.lic ~/
}

#------------------------------------------------------------
# Install system wide dependencies
install_system_dependencies()
{
	logit "Install system dependencies"
	# General update
	sudo apt update
	
	# Get SSH access
	sudo apt -y install openssh-server
	sudo ufw allow ssh
	
	# Network related bits
	sudo apt -y install net-tools
	sudo apt-get install libomp-dev

	# We need libomp in the /usr/local/lib dir for pardiso and ipopt
	sudo cp /usr/lib/x86_64-linux-gnu/libomp5.so ${DEFAULT_LIB_DIR}/libomp.so
}

run_setup()
{
  logit "Running setup"
  install_system_dependencies
  install_ipopt
  install_python
}

check_root()
{
	if [ "$EUID" -ne 0 ]
	then logit "Please run as root/sudo"
		exit
	fi
}


#------------------------------------------------------------
# ENTRY POINT
run_setup