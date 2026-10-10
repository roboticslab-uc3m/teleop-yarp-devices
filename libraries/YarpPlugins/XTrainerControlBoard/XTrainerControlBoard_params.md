| Group | Parameter     | Type        | Units     | Default Value | Required | Description                             | Notes |
|:-----:|:-------------:|:-----------:|:---------:|:-------------:|:--------:|:---------------------------------------:|:-----:|
|       | port          | string      |           | /dev/ttyACM0  | yes      | Name of the serial port                 |       |
|       | baudrate      | int         |           | 2000000       | yes      | Baud rate of the serial port            |       |
|       | motorIds      | vector<int> |           | 1 2 4 5 6 7 8 | yes      | Motor IDs                               |       |
|       | extraId       | int         |           | 3             | yes      | Extra motor ID                          |       |
|       | encoderPulses | int         |           | 4096          | yes      | Number of encoder pulses per revolution |       |
