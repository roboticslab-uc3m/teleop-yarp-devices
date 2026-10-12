| Group | Parameter     | Type           | Units     | Default Value | Required | Description                              | Notes             |
|:-----:|:-------------:|:--------------:|:---------:|:-------------:|:--------:|:----------------------------------------:|:-----------------:|
|       | port          | string         |           | /dev/ttyACM0  | yes      | Name of the serial port                  |                   |
|       | baudrate      | int            |           | 2000000       | yes      | Baud rate of the serial port             |                   |
|       | motorIds      | vector<int>    |           | 1 2 4 5 6 7 8 | yes      | Motor IDs                                |                   |
|       | extraId       | int            |           | 0             | no       | Extra motor ID                           | 0: not used       |
|       | extraReverse  | bool           |           | false         | no       | Whether to reverse the extra motor       |                   |
|       | extraOffset   | double         | deg       | 0.0           | no       | Offset to apply to the extra motor       |                   |
|       | encoderPulses | int            |           | 4096          | yes      | Number of encoder pulses per revolution  | check .model file |
|       | velocityRatio | double         | deg/s     | 1.373329569   | yes      | Factor to apply to velocity measurements | check .model file |
|       | offsets       | vector<double> | deg       | 0 0 0 0 0 0   | no       | Offsets to apply to each motor           |                   |
|       | period        | double         | s         | 0.01          | no       | Period of the encoder acquisition loop   |                   |
