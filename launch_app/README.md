# launch_app
Services and shell scripts to run the OpenSHC, DroneDeploy, and Arachnida ROS packages on bootup.

The services in launch_services must be placed inside the /lib/systemd/system folder on the Jetson which runs Ubunutu 20.04 device. Each service runs a corresponding shell script which launches the required ROS processes.

To check the status of a service, i.e. the rocos-agent which facilitates DroneDeploy communication, run the following command:

`sudo systemctl status rocos-agent`

To start a service, i.e. the openSHC service, run the following command:

`sudo systemctl start cavex_gait`

The systemd logs it messages to the journal process rather than standard output, the following command is useful to determine whether the service start was successful or not and provides indications for fixing service files.

`journalctl -e`

To stop a service run the following command:

`sudo systemctl stop cavex_gait`

To restart OpenSHC run the following bash alias command (defined in ~/.bash_aliases on the Jetson):

`startSHC`

The service files must be located in the /lib/systemd/system directory on the Jetson, once placed there the following command should be run to rebuild the dependency tree. When in doubt try running this command:

`sudo systemctl daemon-reload`

To enable a service, which will cause it to run on system bootup, run the following command:

`sudo systemctl enable cavex_gait`

Running the following command should display whether services starting with identifier cavex are enabled or disabled:

`systemctl list-unit-files cavex*`

To disable a service, i.e. prevent it from executing on system startup, run the following command:

`sudo systemctl disable cavex_gait`

This [resource](https://alesnosek.com/blog/2016/12/04/controlling-a-multi-service-application-with-systemd/) provides good documentation for creating a multi-service application with proper dependency arrangement.

At the end of 2023, the start-up dependency hierarchy for the Jetson Orin NX looked like this:

![systemStartup](https://github.com/CaveX/CaveX2023/assets/110513531/0b690004-c364-4e56-a4dd-eb4afd8cef2e)
