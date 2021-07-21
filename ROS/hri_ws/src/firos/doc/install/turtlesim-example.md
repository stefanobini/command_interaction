# FIROS with Turtlesim and Orion Context-Broker

We will quickly set up a running example of FIROS with the `turtlesim_node`. It is expected, that you already have
installed ROS, turtlesim, a functional Orion Context Broker and followed the Installation instruction explained in the
[Installation-section](../install/install.md) to be able to execute `rosrun firos core.py`.

In this example we will show you how to run two FIROS instances on the same machine looping the messages endlessly
between the Context-Broker and the ROS-World.

## Configuration-Folder for two FIROS instances

Before executing any commands you need to create two configuration folders. In this example we will create two folders
`config_1` and `config_2`. Each of the configuration folder do have the following files with the following content:

`config.json` :

```json
{
    "environment": "test",
    "test": {
        "contextbroker": {
            "address": "localhost",
            "port": 1026
        }
    }
}
```

`whitelist.json` :

```json
{}
```

**NOTE** `config.json`: The Context-Broker runs locally, so the configuration should be fine for you, as long as your
Context-Broker is also locally.

Inside `config_1` we add the file `topics.json` with the following content:

```json
{
    "/turtle1/cmd_vel": ["geometry_msgs/Twist", "publisher"],
    "/turtle1/pose": ["turtlesim/Pose", "subscriber"]
}
```

and for `config_2` the following swapped content:

```json
{
    "/turtle1/cmd_vel": ["geometry_msgs/Twist", "subscriber"],
    "/turtle1/pose": ["turtlesim/Pose", "publisher"]
}
```

Executing `tree` should give you the follwing structure:

```shell
.
├── config_1
│   ├── config.json
│   ├── topics.json
│   └── whitelist.json
└── config_2
    ├── config.json
    ├── topics.json     # Publisher/Subscriber should be swapped here
    └── whitelist.json

```

## Execution of roscore, turtlesim and the Context-Broker

To start the roscore simply exeute:

> roscore

---

To start the Orion Context-Broker (via the docker-compose.yml) execute:

> docker-compose up

You should be able to access `localhost:1026/v2/entities` which should print an empty list.

---

To run turltesim, do the following:

> rosrun turtlesim turtlesim_node

---

You can check whether everything is running fine by executing:

> rostopic list

You should get the following output:

```shell
/rosout
/rosout_agg
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```

## Starting the FIROS instances

Now that everything is set up, you can start a firos instance as follows:

> rosrun firos core.py --conf ./config_1

In this case we need to set a specific configuration folder since we created one specifically for this example.

This creates a topic `/firos` which you can observe via `rostopic list`. You can also check
`localhost:1026/v2/subscriptions` and observe an entry of an subscription of FIROS.

You can even check `localhost:10100` to see that FIROS is telling you that it is actually running.

---

For the second instance of FIROS it is not enough to just execute:

> rosrun firos core.py --conf ./config_2

Since the port `10100` on the machine and the topic `firos` in the ROS-World is already occupied by the other firos
instance.

To execute another instance we need to change those values, which can be done with:

> rosrun firos core.py --conf ./config_2 -P 10101 --ros-node-name firos2

You should now observe that the second instance receives a lot of messages by the Context-Broker. It is also observable
how many messages are sent from the Context-Broker to this FIROS-instance in `localhost:1026/v2/subscriptions`.

---

You can also check out `locahost:1026/v2/entites`. There you can observe the Entity/Robot `turtle1` with its topics
`cmd_vel` and `pose` in json-format

## Neverending loop

**Note:** You can kill the terminal for turtlesim. In this case the two FIROS instances still continue to send the
messages to each other. It is also observable that via `rostopic echo /turtle1/pose` the topic `pose` is still published
through the second FIROS instance. This behaviour is intentional, since for this example we artificially created such a
message loop.

## Demonstration

Below is a quick demonstration of the above described example. On the top left, `roscore`, the Orion-ContextBroker and
Turtlesim is started. On the top-right a Web-Browser shows the content of the Orion-ConextBroker. The console on the
bottom right shows information about the ROS-World. the other two consoles individually start FIROS. At the end:
Turtlesim is stopped and you can observe that the two FIROS-Instances are still sending Messages to each other (see:
[Neverending loop](#Neverending%20loop)).

![Demonstration](../media/turtlesim_example.gif)

**Note:** The small Demonstration above uses the old configuration files (`robots.json`), which was redesigned into
`topics.json`. Also, newer Versions of the Orion Context-Broker do not send updates to Firos, when no data changed.

## Troubleshooting:

### No Messages are sent between the two FIROS-instances

If this is the case. Please check your firewall-configuration (or disable the firewall temporarily, for a quick sanity
check). It can also be a Orion Context-Broker version, which does not send an update to the second firos instance. In
this case, the message is only sent ones.
