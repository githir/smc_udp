# smc_udp
udp communication


### how to use

- Prepare a PC with Autoware installed
- Set the environment so that Autoware can be used.

```
    $ source ~/Autoware/ros/devel/setup.bash
```

- clone repo. from GitHub

```
    $ git clone https://github.com/githir/smc_udp
```
  
- Start roscore

```
    $ roscore
```

- Set the host name or IP address of the receiver

```
    $ rosparam set udp_send_hostname <hostname|IPaddress>
```
 by default, hostname:=127.0.0.1 (localhost)

- open new terminal and start sender

```
    $ source ~/Autoware/ros/devel/setup.bash
    $ cd smc_udp/smc_udp_sender
    $ ./smc_udp_sender.py 
```

- open new terminal and start receiver

```
    $ source ~/Autoware/ros/devel/setup.bash
    $ cd smc_udp/smc_udp_receiver
    $ ./smc_udp_receiver.py 
```

- Publish rostopic and check that sender and receiver are working

```
    $ rostopic pub /smc_cmd autoware_msgs/VehicleCmd "header: ...
```
