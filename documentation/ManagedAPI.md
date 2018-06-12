Managed API
-----------

> Setup a controller and attach servo instances to it. Manipulate servo objects and let the controller synchronize (at a fixed frequency) its "virtual" register values with the real servo devices using a background thread. Beware: this API is more complex to master, and not entirely stable yet ;-)

### Advantages

- Asynchronous API
- Easily handle multiple servo models on the same bus, each servo can have its own settings
- Validate each values sent or read from servos

### Limitations

- Higher level API, you might loose some control on low level communication

## Setup walkthrough

TODO
