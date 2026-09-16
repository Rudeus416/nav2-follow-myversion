#!/bin/bash


DIRNAME=`dirname $0`
#PWD = `pwd`

cd $DIRNAME
source ./ExportAppName.sh

#sed -i "s/export LD_LIBRARY_PATH/#export LD_LIBRARY_PATH/g" ~/.bashrc
source ~/.bashrc
if [ ! -d "/opt/${MVAppName}" ]; then
	echo "Install ${MVAppName},Please wait..."
	mkdir -p /opt/${MVAppName}
	tar -C /opt/${MVAppName} -xzf ./${MVAppName}.tar.gz
else
	echo "Uninstall ${MVAppName},Please wait..."
	rm -rf /opt/${MVAppName}
	echo "Install ${MVAppName},Please wait..."
	mkdir -p /opt/${MVAppName}
	tar -C /opt/${MVAppName} -xzf ./${MVAppName}.tar.gz
fi
#
if [ ! -d "/usr/local/Qt-5.6.3/lib/fonts" ]; then
	mkdir -p /usr/local/Qt-5.6.3/lib/fonts
	cp -r /opt/${MVAppName}/bin/fonts/* /usr/local/Qt-5.6.3/lib/fonts
else
	echo "path exist..."
fi
#

echo "Set up the SDK environment..."

bash $DIRNAME/set_usb_priority.sh
bash $DIRNAME/set_virtualserial_priority.sh
source $DIRNAME/set_env_path.sh /opt/${MVAppName}
source $DIRNAME/set_sdk_version.sh

cd /opt/${MVAppName}/driver/gige
if [ -f /opt/${MVAppName}/driver/gige/unload.sh ]; then
	./unload.sh
fi
if [ -f /opt/${MVAppName}/driver/gige/driver_self_starting.sh ]; then
    /opt/${MVAppName}/driver/gige/driver_self_starting.sh 1 
fi
cd /opt/${MVAppName}/bin
if [ -f /opt/${MVAppName}/bin/script_self_starting.sh ]; then
    /opt/${MVAppName}/bin/script_self_starting.sh 1 
fi

cd /opt/${MVAppName}/logserver
./RemoveServer.sh
./InstallServer.sh


echo "Install ${MVAppName} complete!"
echo "Tips: You should be launch a new terminal or execute source command for the bash environment!"
cd $PWD



