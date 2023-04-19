# ROS2 Humble Ackerman Message to CANBus translator
- Subscribes and reads Ackerman Steering messages from ```/cmd_vel``` topic
- Publishes a translated CANBus message to ```/pub_raw_can```

## Running node
If running on Ubuntu
- ```buildAckermannCanInterfaces.sh``` builds the ROS interfaces
- ```buildAckercan_pubsub.sh``` builds the translator

Once built
- ```runTranslator.sh``` runs the translator
- ```runTalker.sh``` runs a publisher (```/cmd_vel```) for Ackermann messages with random values
- ```runListener.sh``` runs a listener (```/pub_raw_can```) that echoes what it reads
