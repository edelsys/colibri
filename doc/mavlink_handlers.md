#  Implementing mavlink v2 message handlers

Colibri has stackable networking pipeline and provides two ways to work with messages.
For mavlink communication it is highly recommended use table-based method.

Here is an example on how you can add simple mavlink communication:

Declare message handlers
```cpp
fflow::pointprec_t proto_command_handler(uint8_t *, size_t, fflow::SparseAddress, fflow::BaseComponentPtr);
fflow::pointprec_t proto_logging_ack_handler(uint8_t *, size_t, fflow::SparseAddress, fflow::BaseComponentPtr)
```

Setup handlers table
```cpp
fflow::message_handler_note_t proto_table[proto_table_len] = {
    // Setup handler for mavlink message #76
    {MAVLINK_MSG_ID_COMMAND_LONG /* #76 */,
     std::bind(&MutelemetryStreamer::proto_command_handler, this,
               std::placeholders::_1, std::placeholders::_2,
               std::placeholders::_3, std::placeholders::_4)},

    // Setup handler for mavlink message #268
    {MAVLINK_MSG_ID_LOGGING_ACK /* #268 */,
     [this](uint8_t *a, size_t b, fflow::SparseAddress c,
            fflow::BaseComponentPtr d) -> fflow::pointprec_t {
       return proto_logging_ack_handler(a, b, c, d);
     }},
};
```

Set the size of handlers table
```cpp
static constexpr size_t proto_table_len = 2;
```

Finally register handlers within pipeline
```cpp
RouteSystemPtr roster;
...
roster->add_protocol2(proto_table, proto_table_len);
```
